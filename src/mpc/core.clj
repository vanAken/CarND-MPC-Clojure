(ns mpc.core
  (:require [chord.http-kit :refer [with-channel wrap-websocket-handler]]
            [org.httpkit.server :refer [run-server]]
            [clojure.core.async :refer [<! >! go-loop chan dropping-buffer]]
            [clojure.data.json :as json]
            [clojure.string :refer [index-of last-index-of]]
            [figurer.core :as figurer]
            [incanter.distributions :refer [uniform-distribution]]
            [frenet.core :as frenet])
  (:gen-class))

(defn convert-point-to-vehicle-frame
  "Convert a point from absolute coordinates to vehicle reference frame"
  [absxy carxy carpsi]
  (let [shift_x (- (absxy 0) (carxy 0)) 
        shift_y (- (absxy 1) (carxy 1)) 
        cos-psi (Math/cos (- carpsi))
        sin-psi (Math/sin (- carpsi))
        relx (- (* shift_x cos-psi)(* shift_y sin-psi))
        rely (+ (* shift_x sin-psi)(* shift_y cos-psi))]
    [relx rely]))

(defn convert-points-to-vehicle-frame
  "Convert list of x and list of y from absolute coordinates to vehicle reference frame"
  [absx-list absy-list carxy carpsi]
  (mapv #(convert-point-to-vehicle-frame [%1 %2] carxy carpsi) absx-list absy-list))

; Udacity requires 100 ms delay when submitting.
; Feel free to experiment with other values until then.
(def actuation-period-milliseconds 40)

(def speed 100)
(def Lf 2.67)
(def depth 5)

(defn policy
  "Given current state, determine next actuation.
   Each element of the result vector is a probability
   distribution to represent the uncertainty in which
   actuation would be the best."
  [state]
  (let [[x y psi v vx vy s d vs vd delta ddelta] state 
        steering 0.0 ; TODO: Use pd-steering-estimate for better guess.
        throttle 1.0]
    [(uniform-distribution (- steering 0.4) (+ steering 0.4))
     (uniform-distribution (- throttle 0.0) (+ throttle 0.0))]))

(defn predict
  "Given current state and actuation, determine how
   the state will change."
  [state actuation coord dt]
  (let [[x0 y0 psi0 v0 vx0 vy0 s d vs vd delta0 ddelta] state
        [steering throttle] actuation
        steering_radians (* 25 steering (/ Math/PI 180))
        delta1 (* v0 dt (/ steering_radians Lf))
        ddelta (- delta1 delta0)
        ; Physics
        x1 (+ x0 (* vx0 dt)) ; TODO: Calculate new value of x based on x0.
        y1 (+ y0 (* vy0 dt)) ; TODO: Calculate new value of y based on y0.
        psi1 (- psi0 delta1)
        v1 (+ v0 (* throttle dt)) 
        ; Derived parts of state
        vx1 (* v1 (Math/cos psi1))
        vy1 (* v1 (Math/sin psi1))
        [s d vs vd] (frenet/xyv->sdv coord x1 y1 vx1 vy1)]
    [x1 y1 psi1 v1 vx1 vy1 s d vs vd delta1 ddelta]))

(defn value
  "Measure of how 'good' a state is. A plan will
  be chosen that maximizes the average result of
  this function across each state in the plan."
  [state coords dt]
  (let [[x y psi v vx vy s d vs vd delta ddelta] state
        [x1 y1] (frenet/sd->xy coords 0 0)
        [dx dy] [(- x1 x)(- y1 y)]
        Euclidean_Distance2 (+ (* dx dx)(* dy dy))]
        ;Euclidean_Distance (Math/sqrt Euclidean_Distance2)
        ;_ (println "Euclidean_Distance" Euclidean_Distance2 (+ (* d d)(* s s)))] 
    (+ ; Reference State Costs
       (*   -1000 1000 d d  d d  d d)           ; distance from center
       ;(*   -10 (/ d s) (/ d s))))) ; <= simlyfyedewpsi = atan(d/s)
       ;(*   -1 speed-to-max))))
       (*   -1000 vd vd  vd vd)              ; wobbling
       (*   +1 Euclidean_Distance2 Euclidean_Distance2) ; Euclidean Distance
       (*   -1000 1000 delta delta  delta delta)       ; Steering 
       (*   -1 ddelta ddelta)))) ; Change in steering

(defn controller
  "Given telemetry (information about vehicle's situation)
  decide actuation (steering angle and throttle)."
  [telemetry]
  (let [rel-waypoints (convert-points-to-vehicle-frame
                        (:ptsx telemetry) (:ptsy telemetry)
                        [(:x telemetry) (:y telemetry)]
                        (:psi telemetry))
        [x y psi] [0 0 0]
        [vx vy] [(:speed telemetry) 0]
        coord (frenet/track rel-waypoints)
        [s d vs vd] (frenet/xyv->sdv coord x y vx vy)
        ini_state [x y psi vx vx vy s d vs vd 0 0]
        
        dt (* 0.001 actuation-period-milliseconds) ;delay-seconds
        state (predict ini_state 
                [(:steering-angle telemetry) (:throttle telemetry)]
                coord
                dt)
        _ (println "speed vs:" vx "throttle a:" (:throttle telemetry))
        problem (figurer/define-problem {:policy policy  
                                         :predict (fn [state actuation] 
                                                    (predict state actuation coord dt))
                                         :value   (fn [state] 
                                                    (value state coord dt))
                                         :initial-state state
                                         :depth depth}); TODO: Choose depth - how many steps ahead figurer should explore.   
        solution (figurer/figure problem
                   {:max-seconds dt})
        plan (figurer/sample-plan solution)
        plan-value (figurer/expected-value solution)
        [steering throttle] (first (:actuations plan))
        plan-xy (mapv #(vec (take 2 %)) (:states plan))
        result {:steering-angle steering
                :throttle throttle
                :waypoints rel-waypoints
                :plan plan-xy}]
    result))

(defn format-actuation
  "Format actuation (:steering-angle and :throttle) for transmission to simulator."
  [{:keys [steering-angle throttle waypoints plan] :as actuation}]
  (let [[way-x way-y] (apply mapv vector waypoints)
        [plan-x plan-y] (apply mapv vector plan)]
    (str "42"
      (json/write-str
        ["steer"
         {"steering_angle" steering-angle
          "throttle" throttle
          "next_x" way-x
          "next_y" way-y
          "mpc_x" plan-x
          "mpc_y" plan-y}]))))

(defn parse-message
  "Parse message from Udacity's SDC term 2 simulator for the PID project."
  [msg]
  (if (and msg (> (.length msg) 2) (= (subs msg 0 2) "42"))
    (let [json-start (index-of msg "[")
          json-end   (last-index-of msg "]")
          json-str   (subs msg json-start (inc json-end))
          json-msg   (json/read-str json-str)]
      (if (= (get json-msg 0) "telemetry")
        (let [data (get json-msg 1)]
          (if data
            {:type :telemetry
             :ptsx (get data "ptsx")
             :ptsy (get data "ptsy")
             :x (get data "x")
             :y (get data "y")
             :speed (get data "speed")
             :psi (get data "psi")
             :psi-unity (get data "psi_unity")
             :steering-angle (get data "steering_angle")
             :throttle (get data "throttle")}
            {:type :manual}))
        json-msg))
    nil))

(defn handler
  "Called in response to websocket connection. Handles sending and receiving messages."
  [{:keys [ws-channel] :as req}]
  (go-loop []
    (let [{:keys [message]} (<! ws-channel)
          parsed (parse-message message)]
      (when parsed
        (when (= :telemetry (:type parsed))
          (let [start-millis (.getTime (java.util.Date.))
                response (format-actuation (controller parsed))
                end-millis (.getTime (java.util.Date.))
                millis-remaining (- actuation-period-milliseconds
                                    (- end-millis start-millis))]
            (when (> millis-remaining 0)
              (Thread/sleep millis-remaining))
            (>! ws-channel response))))
      (when (= :manual (:type parsed))
        (Thread/sleep actuation-period-milliseconds)
        (>! ws-channel "42[\"manual\",{}]")))
    (recur)))

(defn -main
  "Run websocket server to communicate with Udacity MPC simulator."
  [& args]
  (println "Starting server")
  (run-server (-> #'handler
                  (wrap-websocket-handler
                    {:read-ch (chan (dropping-buffer 10))
                     :format :str}))
    {:port 4567}))

