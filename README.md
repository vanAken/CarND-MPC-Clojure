# CarND Model Predictive Control
[![Udacity - Self-Driving Car NanoDegree](https://s3.amazonaws.com/udacity-sdc/github/shield-carnd.svg)](http://www.udacity.com/drive)

Self-Driving Car Engineer Nanodegree Program

---

Clojure version of Udacity's MPC project from term 2 of the self-driving
car engineer nanodegree. This repository is intended to serve as starter code for
other students who wish to complete the project in Clojure.

## Why Clojure?

The most common choices for self-driving car development are C++ and Python.
[Clojure](https://clojure.org/)
supports a faster development style than either of these languages (especially C++).
Compared to C++, Clojure has a much simpler and more flexible syntax, clear
error handling, and sophisticated dependency management. Compared to Python, Clojure is
much faster (close to C++) and has excellent concurrency support.

Here's a [tutorial](https://www.maria.cloud/) to help you get started.

## Installation

You will neeed to install
[Leiningen](https://leiningen.org/),
a Clojure build tool. This is a fairly easy
installation process. Just follow the instructions on the
[Leiningen](https://leiningen.org/) website.

I also recommend [VS Code](https://code.visualstudio.com/) with the
[Calva extension](https://marketplace.visualstudio.com/items?itemName=cospaia.clojure4vscode)
as your first Clojure text editor because it is very easy to install
and use. Later, you can explore more advanced options like
[Cursive (IntelliJ)](https://cursive-ide.com/),
[CIDER (Emacs)](https://github.com/clojure-emacs/cider),
or [Vim](https://github.com/tpope/vim-fireplace).

## Usage

You'll find many TODO comments in src/mpc/core.clj indicating parts of
the code that you will need to complete. The code already runs as-is,
but the car will drive poorly until you make improvements.

You can run the code with the following command. You should also run
[Udacity's term 2 simulator](https://github.com/udacity/self-driving-car-sim/releases)
at the same time and select the "MPC Control" project.

    $ lein run

The idea of model predictive control is to describe the problem and desired outcome,
then let an optimization library find a good solution to that problem. In this case,
we use the [figurer](https://github.com/ericlavigne/figurer) library to perform the
optimization. You will need to inform figurer about what kind of outcome you want
(value function), the available actions and their likelihood of each action (policy
function), and the mechanics of this problem (prediction function). Start with
simple versions of each of these functions, then enhance these functions to improve
your car's steering. For added challenge, see how fast you can go without leaving
the track. (Over 100 MPH is possible!)

## The result compare to the c++ version

The state variables from the MPC lesson are:

x, y, ψ v cte and eψ

and changed into:

x, y, ψ v vx vy s d vs vd δ dδ in line 47 – 65.

Also I use frenet to convert from x y vx vy into local frenet d s vd vs (line 64, 93)
and all waypoints ( line 92).

The model is used for the initial state and every predition step in line 47 – 65 of core.clj.

x [t+1] = x [t] + vx [t] * dt

y [t+1] = y [t] + vy [t] * dt

ψ [t+1] = ψ [t] – v [t] * δ /Lf * dt

v [t+1] = v [t] + a * dt

vx [t+1] = vx [t] cos ψ [t]

yx [t+1] = y x [t] sin ψ [t]

[s d vs vd] = frenet/xyv→sdv ( [global coordinates], x, y, vx, vy )

Where d is -cte and eψ proportional to vd.

Instead of ipopt I used figure in a very early "crude" version 0.1.0.
But the results are faster than my C++ version. 

Figure is a MCTS - “Monte Carlo Tree Search” algorithm. You can found some details here.
[figurer](https://github.com/ericlavigne/figurer)
In the clojure solution (line 80-86 of core.clj) the translational and the rotational error has to minimize,
while the the Euclidean Distance has to maximize. In additional the steering cost and the change of
steering was used as well.
N was choose to only 4 and dt was also 0.1s. Figure has a police which use probability distribution for the
actuators, which represent the uncertainty for the best solution (line 36-49 of core.clj).

## This is how it looks like:
![Car goes with over 100 mph around the around the well know curve after the bridge:](./MPC_clojure_100_plus_50ms.gif)
