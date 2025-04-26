<div align="center">


<picture>
  <source media="(prefers-color-scheme: dark)" srcset="https://readme-typing-svg.demolab.com?font=Mona+Sans&size=28&weight=600&duration=2500&pause=1000&color=FFFFFF&width=435&center=true&lines=I'm+Philipp+%F0%9F%91%8B;I'm++a+Roboticist+%F0%9F%A4%96;I'm++a+Programmer+%F0%9F%92%BB;I'm++a+Researcher+%F0%9F%94%AC">
  <img src="https://readme-typing-svg.demolab.com?font=Mona+Sans&size=28&weight=600&duration=2500&pause=1000&color=000000&width=435&center=true&lines=I'm+Philipp+%F0%9F%91%8B;I'm++a+Roboticist+%F0%9F%A4%96;I'm++a+Programmer+%F0%9F%92%BB;I'm++a+Researcher+%F0%9F%94%AC" />
</picture>
</div>


# About

<p dir="rtl" align="right">
<i><b>What I cannot create, I do not understand</b> - Richard Feynman</i>
</p>


<details>

<summary>
I'm a robotics engineer with focus on real-time systems and a passion for elegant and scalable solutions to real-world problems.
</summary>

<br>
<p align="right">
Automation fascinated me ever since I played with Lego as a little child.<br>
However, by experience I learned that the best solutions are usually the simple ones.<br>
Not the easy ones, but the simple, elegant, readable yet impressive ones.
</p>
<br>

I've done my PhD with the "Robotics and Perception Group" led by Prof. Davide Scaramuzza at the University of Zurich. My research is focused on control systems for vision-based aerial vehicles with prospects towards machine learning.

In my master studies at the ETH Zürich in "Robotics, Systems and Control" I also worked with autonomous vehicles and electric drive systems. I've completed my master thesis on model predictive control for quadrotors and a semester thesis on interaction with quadrotors.

During my bachelor in "Mechanical and Process Engineering" I participated in the "Formula Student Electric" project at the ETH Zürich with the "academic motorsport association Zürich", short AMZ. For two consecutive years I could gather experience in team work under time pressure, first as an automotive engineer in control systems and then as CTO leading the electrical development of a race car.
I've extended this experience in my master by supervising a team of 10 students working on an autonomous race car for demonstration purposes, developed in a one-year project under the lead of Prof. Roland Siegwart.
</details>

# :rocket: Work

> Senior Autonomy Software Engineer with PhD in Robotics at [Skydio](skydio.com)


# :microscope: Publications

### First and shared authorship:

<details>
<summary>
<b>Science Robotics'21</b>: <i><a href="http://rpg.ifi.uzh.ch/docs/ScienceRobotics21_Foehn.pdf">Time-Optimal Planning for Quadrotor Waypoint Flight</a></i>
</summary>

Philipp Foehn, Angel Romero, Davide Scaramuzza

Science Robotics, 2021-05-21

* Paper: http://rpg.ifi.uzh.ch/docs/ScienceRobotics21_Foehn.pdf
* Video: https://youtu.be/ZPI8U1uSJUs
* Code: https://github.com/uzh-rpg/rpg_time_optimal

Quadrotors are amongst the most agile flying robots.
However, planning time-optimal trajectories at the actuation limit through multiple waypoints remains an open problem.
This is crucial for applications such as inspection, delivery, search and rescue, and drone racing.
Early works used polynomial trajectory formulations, which do not exploit the full actuator potential due to their inherent smoothness.
Recent works resorted to numerical optimization, but require waypoints to be allocated as costs or constraints at specific discrete times.
However, this time- allocation is a priori unknown and renders previous works incapable of producing truly time-optimal trajectories.
To generate truly time-optimal trajectories, we propose a solution to the time allocation problem while exploiting the full quadrotor’s actuator potential.
We achieve this by introducing a formulation of progress along the trajectory, which enables the simultaneous optimization of the time- allocation and the trajectory itself.
We compare our method against related approaches and validate it in real- world flights in one of the world’s largest motion-capture systems, where we outperform human expert drone pilots in a drone-racing task.
</details>

<details>
<summary>
<b>Science Robotics'21</b>: <i><a href="http://rpg.ifi.uzh.ch/docs/ScienceRobotics21_Foehn.pdf">Agilicious: Open-Source and Open-Hardware Agile Quadrotor for Vision-Based Flight</a></i>
</summary>

Philipp Foehn, Elia Kaufmann, Angel Romero, Robert Penicka, Sihao Sun, Leonard Bauersfeld, Thomas Laengle, Giovanni Cioffi, Yunlong Song, Antonio Loquercio, Davide Scaramuzza

Science Robotics, 2021-12-22

* Paper: http://rpg.ifi.uzh.ch/docs/ScienceRobotics21_Foehn.pdf
* Code: https://agilicious.dev

Autonomous, agile quadrotor flight raises fundamental challenges for robotics research in terms of perception, planning, learning, and control.
A versatile and standardized platform is needed to accelerate research and let practitioners focus on the core problems.
To this end, we present Agilicious, a co-designed hardware and software framework tailored to autonomous, agile quadrotor flight.
It is completely open-source and open-hardware and supports both model-based and neural-network--based controllers.
Also, it provides high thrust-to-weight and torque-to-inertia ratios for agility, onboard vision sensors, GPU-accelerated compute hardware for real-time perception and neural-network inference, a real-time flight controller, and a versatile software stack.
In contrast to existing frameworks, Agilicious offers a unique combination of flexible software stack and high-performance hardware.
We compare Agilicious with prior works and demonstrate it on different agile tasks, using both model-based and neural-network--based controllers.
Our demonstrators include trajectory tracking at up to 5g and 70km/h in a motion-capture system, and vision-based acrobatic flight and obstacle avoidance in both structured and unstructured environments using solely onboard perception.
Finally, we  demonstrate its use for hardware-in-the-loop simulation in virtual-reality environments.
Thanks to its versatility, we believe that Agilicious supports the next generation of scientific and industrial quadrotor research.
</details>

<details>
<summary>
<b>RSS'20</b>: <i><a href="http://rpg.ifi.uzh.ch/docs/RSS20_Foehn.pdf">AlphaPilot: Autonomous Drone Racing</a></i>
</summary>

Philipp Foehn, Dario Brescianini, Elia Kaufmann, Titus Cieslewski, Mathias Gehrig, Manasi Muglikar, Davide Scaramuzza

Robotics: Science and Systems, RSS, 2020-07-01


* Paper: http://rpg.ifi.uzh.ch/docs/RSS20_Foehn.pdf
* Pitch: https://youtu.be/ZIHjswKDods
* Video: https://youtu.be/DGjwm5PZQT8

This paper presents a novel system for autonomous, vision-based drone racing combining learned data abstraction, nonlinear filtering, and time-optimal trajectory planning.
The system has successfully been deployed at the first autonomous drone racing world championship: the 2019 AlphaPilot Challenge.
Contrary to traditional drone racing systems, which only detect the next gate, our approach makes use of any visible gate and takes advantage of multiple, simultaneous gate detections to compensate for drift in the state estimate and build a global map of the gates.
The global map and drift-compensated state estimate allow the drone to navigate through the race course even when the gates are not immediately visible and further enable to plan a near time-optimal path through the race course in real time based on approximate drone dynamics.
The proposed system has been demonstrated to successfully guide the drone through tight race courses reaching speeds up to 8 m/s and ranked second at the 2019 AlphaPilot Challenge.
</details>

<details>
<summary>
<b>RSS'19</b>: <i><a href="http://rpg.ifi.uzh.ch/docs/RSS19_Nisar.pdf">VIMO: Simultaneous Visual Inertial Model-based Odometry and Force Estimation</a></i>
</summary>

Barza Nisar, Philipp Foehn, Davide Falanga, Davide Scaramuzza

Robotics: Science and Systems, RSS, 2019-06-22

* Paper: http://rpg.ifi.uzh.ch/docs/RSS19_Nisar.pdf
* Code: https://github.com/uzh-rpg/vimo
* Video: https://youtu.be/t2GdZZp7xQE

In recent years, many approaches to Visual Inertial Odometry (VIO) have become available.
However, they neither exploit the robot’s dynamics and known actuation inputs, nor differentiate between desired motion due to actuation and unwanted perturbation due to external force.
For many robotic applications, it is often essential to sense the external force acting on the system due to, for example, interactions, contacts, and disturbances.
Adding a motion constraint to an estimator leads to a discrepancy between the model-predicted motion and the actual motion.
Our approach exploits this discrepancy and resolves it by simultaneously estimating the motion and the external force.
We propose a relative motion constraint combining the robot’s dynamics and the external force in a preintegrated residual, resulting in a tightly-coupled, sliding-window estimator exploiting all correlations among all variables.
We implement our Visual Inertial Model-based Odometry (VIMO) system into a state-ofthe-art VIO approach and evaluate it against the original pipeline without motion constraints on both simulated and real-world data.
The results show that our approach increases the accuracy of the estimator up to 29% compared to the original VIO, and provides external force estimates at no extra computational cost.
To the best of our knowledge, this is the first approach exploiting model dynamics by jointly estimating motion and external force.
Our implementation will be made available open-source.
</details>



<details>
<summary>
<b>IROS'18</b>: <i><a href="http://rpg.ifi.uzh.ch/docs/IROS18_Falanga.pdf">PAMPC: Perception-Aware Model Predictive Control for Quadrotors</a></i>  
</summary>

Davide Falanga, Philipp Foehn, Peng Lu, Davide Scaramuzza

IEEE/RSJ International Conference on Intelligent Robots and Systems, IROS, 2018-10-01

* PDF: http://rpg.ifi.uzh.ch/docs/IROS18_Falanga.pdf
* Code: https://github.com/uzh-rpg/rpg_quadrotor_mpc
* Video: https://www.youtube.com/watch?v=9vaj829vE18

We present the first perception-aware model predictive control framework for quadrotors that unifies control and planning with respect to action and perception objectives.
Our framework leverages numerical optimization to compute trajectories that satisfy the system dynamics and require control inputs within the limits of the platform.
Simultaneously, it optimizes perception objectives for robust and reliable sensing by maximizing the visibility of a point of interest and minimizing its velocity in the image plane.
Considering both perception and action objectives for motion planning and control is challenging due to the possible conflicts arising from their respective requirements.
For example, for a quadrotor to track a reference trajectory, it needs to rotate to align its thrust with the direction of the desired acceleration.
However, the perception objective might require to minimize such rotation to maximize the visibility of a point of interest.
A model-based optimization framework, able to consider both perception and action objectives and couple them through the system dynamics, is therefore necessary.
Our perception-aware model predictive control framework works in a receding-horizon fashion by iteratively solving a non-linear optimization problem.
It is capable of running in real-time, fully onboard our lightweight, small-scale quadrotor using a low-power ARM computer, together with a visual-inertial odometry pipeline.
We validate our approach in experiments demonstrating (I) the conflict between perception and action objectives, and (II) improved behavior in extremely challenging lighting conditions.

</details>



<details>
<summary>
<b>ICRA'18</b>: <i><a href="http://rpg.ifi.uzh.ch/docs/ICRA18_Foehn.pdf">Onboard State Dependent LQR for Agile Quadrotors</a></i>
</summary>

Philipp Foehn, Davide Scaramuzza

IEEE International Conference on Robotics and Automation, ICRA, 2018-05-20


* Paper: http://rpg.ifi.uzh.ch/docs/ICRA18_Foehn.pdf
* Slides: http://rpg.ifi.uzh.ch/docs/ICRA18_Foehn.pptx
* Video: https://youtu.be/8OVsJNgNfa0
* ICRA Video Pitch: https://youtu.be/c7gHF-NJjPo

State-of-the-art approaches in quadrotor control split the problem into multiple cascaded subproblems, exploiting the different time scales of the rotational and translational dynamics.
They calculate a desired acceleration as input for a cascaded attitude controller but omit the attitude dynamics.
These approaches use limits on the desired acceleration to maintain feasibility and robustness through the control cascade.
We propose an implementation of an LQR controller, which: (I) is linearized depending on the quadrotor’s state; (II) unifies the control of rotational and translational states; (III) handles time-varying system dynamics and control parameters.
Our implementation is efficient enough to compute the full linearization and solution of the LQR at a minimum of 10 Hz on the vehicle using a common ARM processor.
We show four successful experiments: (I) controlling at hover state with large disturbances; (II) tracking along a trajectory; (III) tracking along an infeasible trajectory; (IV) tracking along a trajectory with disturbances.
All the experiments were done using only onboard visual inertial state estimation and LQR computation.
To the best of our knowledge, this is the first implementation and evaluation of a state-dependent LQR capable of onboard computation while providing this amount of versatility and performance.
</details>

<details>
<summary>
<b>RSS'17</b>: <i><a href="">Fast Trajectory Optimization for Agile Quadrotor Maneuvers with a Cable-Suspended Payload</a></i>
</summary>

Philipp Foehn, Davide Falanga, Naveen Kuppuswamy, Russ Tedrake, Davide Scaramuzza

Robotics: Science and Systems, RSS, 2017-07-01

* Paper: http://rpg.ifi.uzh.ch/docs/RSS17_Foehn.pdf
* Slides: http://rpg.ifi.uzh.ch/docs/RSS17_Foehn.pptx
* Video: https://www.youtube.com/watch?v=s9zb5MRXiHA

Executing agile quadrotor maneuvers with cablesuspended payloads is a challenging problem and complications induced by the dynamics typically require trajectory optimization.
State-of-the-art approaches often need significant computation time and complex parameter tuning.
We present a novel dynamical model and a fast trajectory optimization algorithm for quadrotors with a cable-suspended payload.
Our first contribution is a new formulation of the suspended payload behavior, modeled as a link attached to the quadrotor with a combination of two revolute joints and a prismatic joint, all being passive.
Differently from state of the art, we do not require the use of hybrid modes depending on the cable tension.
Our second contribution is a fast trajectory optimization technique for the aforementioned system.
Our model enables us to pose the trajectory optimization problem as a Mathematical Program with Complementarity Constraints (MPCC).
Desired behaviors of the system (e.g., obstacle avoidance) can easily be formulated within this framework.
We show that our approach outperforms the state of the art in terms of computation speed and guarantees feasibility of the trajectory with respect to both the system dynamics and control input saturation, while utilizing far fewer tuning parameters.
We experimentally validate our approach on a real quadrotor showing that our method generalizes to a variety of tasks, such as flying through desired waypoints while avoiding obstacles, or throwing the payload toward a desired target.
To the best of our knowledge, this is the first time that three-dimensional, agile maneuvers exploiting the system dynamics have been achieved on quadrotors with a cable-suspended payload.
</details>




### Co-authorship:


<details>
<summary>
<b>ICRA'22</b>: <i><a href="https://rpg.ifi.uzh.ch/docs/ICRA22_Nan.pdf">Nonlinear MPC for Quadrotor Fault-Tolerant Control</a></i>
</summary>

Fang Nan, Sihao Sun, Philipp Foehn, Davide Scaramuzza

IEEE International Conference on Robotics and Automation, ICRA, 2022-01-23


* Paper: https://rpg.ifi.uzh.ch/docs/ICRA22_Nan.p
* Video: https://youtu.be/Cn_836XGEnU


The mechanical simplicity, hover capabilities, and high agility of quadrotors lead to a fast adaption in the industry for inspection, exploration, and urban aerial mobility.
On the other hand, the unstable and underactuated dynamics of quadrotors render them highly susceptible to system faults, especially rotor failures.
In this work, we propose a fault-tolerant controller using the nonlinear model predictive control (NMPC) to stabilize and control a quadrotor subjected to the complete failure of a single rotor.
Differently from existing works that either rely on linear assumptions or resort to cascaded structures neglecting input constraints in the outer-loop, our method leverages full nonlinear dynamics of the damaged quadrotor and considers the thrust constraint of each rotor.
Hence, this method can seamlessly transition from nominal to rotor failure flights, and effectively perform upset recovery from extreme initial conditions.
Extensive simulations and real-world experiments are conducted for validation, which demonstrates that the proposed NMPC method can effectively recover the damaged quadrotor even if the failure occurs during aggressive maneuvers, such as flipping and tracking agile trajectories.
</details>


<details>
<summary>
<b>TRO'22</b>: <i><a href="http://rpg.ifi.uzh.ch/docs/ICRA21_Sun.pdf">Model Predictive Contouring Control for Near-Time-Optimal Quadrotor Flight</a></i>
</summary>

Angel Romero, Sihao Sun, Philipp Foehn, Davide Scaramuzza

IEEE Transactions on Robotics, TRO, 2022-10-01

* Paper: http://rpg.ifi.uzh.ch/docs/Arxiv21_MPCC_Romero.pdf
* Video: https://youtu.be/mHDQcckqdg4

We tackle the problem of flying time-optimal trajectories through multiple waypoints with quadrotors.
State-of-the-art solutions split the problem into a planning task - where a global, time-optimal trajectory is generated - and a control task - where this trajectory is accurately tracked.
However, at the current state, generating a time-optimal trajectory that takes the full quadrotor model into account is computationally demanding (in the order of minutes or even hours).
This is detrimental for replanning in presence of disturbances.
We overcome this issue by solving the time-optimal planning and control problems concurrently via Model Predictive Contouring Control (MPCC).
Our MPCC optimally selects the future states of the platform at runtime, while maximizing the progress along the reference path and minimizing the distance to it.
We show that, even when tracking simplified trajectories, the proposed MPCC results in a path that approaches the true time-optimal one, and which can be generated in real-time.
We validate our approach in the real-world, where we show that our method outperforms both the current state-of-the-art and a world-class human pilot in terms of lap time achieving speeds of up to 60 km/h.
</details>

<details>
<summary>
<b>TRO'21</b>: <i><a href="https://rpg.ifi.uzh.ch/docs/TRO22_Sun.pdf">A Comparative Study of Nonlinear MPC and Differential-Flatness-Based Control for Quadrotor Agile Flight</a></i>
</summary>

Sihao Sun, Angel Romero, Philipp Foehn, Elia Kaufmann, Davide Scaramuzza

IEEE Transactions on Robotics, TRO, 2021-09-06

* Paper: https://rpg.ifi.uzh.ch/docs/TRO22_Sun.pdf
* Video: https://youtu.be/XpuRpKHp_Bk

Accurate trajectory tracking control for quadrotors is essential for safe navigation in cluttered environments.
However, this is challenging in agile flights due to nonlinear dynamics, complex aerodynamic effects, and actuation constraints.
In this article, we empirically compare two state-of-the-art control frameworks: the nonlinear-model-predictive controller (NMPC) and the differential-flatness-based controller (DFBC), by tracking a wide variety of agile trajectories at speeds up to 72 km/h.
The comparisons are performed in both simulation and real-world environments to systematically evaluate both methods from the aspect of tracking accuracy, robustness, and computational efficiency.
We show the superiority of NMPC in tracking dynamically infeasible trajectories, at the cost of higher computation time and risk of numerical convergence issues.
For both methods, we also quantitatively study the effect of adding an inner-loop controller using the incremental nonlinear dynamic inversion (INDI) method, and the effect of adding an aerodynamic drag model.
Our real-world experiments, performed in one of the world's largest motion capture systems, demonstrate more than 78% tracking error reduction of both NMPC and DFBC, indicating the necessity of using an inner-loop controller and aerodynamic drag model for agile trajectory tracking.
</details>

<details>
<summary>
<b>RA-L'21</b>: <i><a href="http://rpg.ifi.uzh.ch/docs/RAL21_Torrente.pdf">Data-Driven MPC for Quadrotors</a></i>
</summary>

Guillem Torrente, Elia Kaufmann, Philipp Foehn, Davide Scaramuzza

IEEE Robotics and Automation Letters, RA-L, 2021-02-02


* Paper: http://rpg.ifi.uzh.ch/docs/RAL21_Torrente.pdf
* Code: https://github.com/uzh-rpg/fault_tolerant_control
* Video: https://youtu.be/FHvDghUUQtc

Aerodynamic forces render accurate high-speed trajectory tracking with quadrotors extremely challenging.
These complex aerodynamic effects become a significant disturbance at high speeds, introducing large positional tracking errors, and are extremely difficult to model.
To fly at high speeds, feedback control must be able to account for these aerodynamic effects in real-time.
This necessitates a modeling procedure that is both accurate and efficient to evaluate.
Therefore, we present an approach to model aerodynamic effects using Gaussian Processes, which we incorporate into a Model Predictive Controller to achieve efficient and precise real-time feedback control, leading to up to 70% reduction in trajectory tracking error at high speeds.
We verify our method by extensive comparison to a state-of-the- art linear drag model in synthetic and real-world experiments at speeds of up to 14m/s and accelerations beyond 4g.
</details>


<details>
<summary>
<b>RA-L'21</b>: <i><a href="http://rpg.ifi.uzh.ch/docs/RAL21_Hanover.pdf">Performance, Precision, and Payloads: Adaptive Nonlinear MPC for Quadrotors</a></i>
</summary>

Drew Hanover, Philipp Foehn, Sihao Sun, Elia Kaufmann, Davide Scaramuzza

IEEE Robotics and Automation Letters, RA-L, 2021-12-01

* Paper: http://rpg.ifi.uzh.ch/docs/RAL21_Hanover.pdf
* Video: https://youtu.be/8oB1rG5iYc4


Agile quadrotor flight in challenging environments has the potential to revolutionize shipping, transportation, and search and rescue applications.
Nonlinear model predictive control (NMPC) has recently shown promising results for agile quadrotor control, but relies on highly accurate models for maximum performance.
Hence, model uncertainties in the form of unmodeled complex aerodynamic effects, varying payloads and parameter mismatch will degrade overall system performance.
In this paper, we propose L1-NMPC, a novel hybrid adaptive NMPC to learn model uncertainties online and immediately compensate for them, drastically improving performance over the non-adaptive baseline with minimal computational overhead.
Our proposed architecture generalizes to many different environments from which we evaluate wind, unknown payloads, and highly agile flight conditions.
The proposed method demonstrates immense flexibility and robustness, with more than 90% tracking error reduction over non-adaptive NMPC under large unknown disturbances and without any gain tuning.
In addition, the same controller with identical gains can accurately fly highly agile racing trajectories exhibiting top speeds of 70 km/h, offering tracking performance improvements of around 50% relative to the non-adaptive NMPC baseline.
</details>


<details>
<summary>
<b>RSS'21</b>: <i><a href="http://rpg.ifi.uzh.ch/docs/RSS21_Bauersfeld.pdf">NeuroBEM: Hybrid Aerodynamic Quadrotor Model</a></i>
</summary>

Leonard Bauersfeld, Elia Kaufmann, Philipp Foehn, Sihao Sun, Davide Scaramuzza

Robotics: Science and Systems, RSS, 2021-06-15

* Paper: http://rpg.ifi.uzh.ch/docs/RSS21_Bauersfeld.pdf
* Code: http://rpg.ifi.uzh.ch/NeuroBEM.html
* Video: https://www.youtube.com/watch?v=Nze1wlfmzTQ

Quadrotors are extremely agile, so much in fact, that classic first-principle-models come to their limits.
Aerodynamic effects, while insignificant at low speeds, become the dominant model defect during high speeds or agile maneuvers.
Accurate modeling is needed to design robust high-performance control systems and enable flying close to the platform’s physical limits.
We propose a hybrid approach fusing first principles and learning to model quadrotors and their aerodynamic effects with unprecedented accuracy.
First principles fail to capture such aerodynamic effects, rendering traditional approaches inaccurate when used for simulation or controller tuning.
Data-driven approaches try to capture aerodynamic effects with blackbox modeling, such as neural networks; however, they struggle to robustly generalize to arbitrary flight conditions.
Our hybrid approach unifies and outperforms both first-principles blade-element momentum theory and learned residual dynamics.
It is evaluated in one of the world’s largest motion-capture systems, using autonomous-quadrotor-flight data at speeds up to 65 km/h.
The resulting model captures the aerodynamic thrust, torques, and parasitic effects with astonishing accuracy, outperforming existing models with 50% reduced prediction errors, and shows strong generalization capabilities beyond the training set.
</details>

<details>
<summary>
<b>ICRA'19</b>: <i><a href="http://rpg.ifi.uzh.ch/docs/ICRA19_Kaufmann.pdf">Beauty and the Beast: Optimal Methods Meet Learning for Drone Racing</a></i>  
</summary>

Elia Kaufmann, Mathias Gehrig, Philipp Foehn, Rene Ranftl, Alexey Dosovitskiy, Vladlen Koltun, Davide Scaramuzza
IEEE International Conference on Robotics and Automation, ICRA, 2019-05-20


* Paper: http://rpg.ifi.uzh.ch/docs/ICRA19_Kaufmann.pdf
* Video: https://youtu.be/UuQvijZcUSc


Autonomous micro aerial vehicles still struggle with fast and agile maneuvers, dynamic environments, imperfect sensing, and state estimation drift.
Autonomous drone racing brings these challenges to the fore.
Human pilots can fly a previously unseen track after a handful of practice runs.
In contrast, state-of-the-art autonomous navigation algorithms require either a precise metric map of the environment or a large amount of training data collected in the track of interest.
To bridge this gap, we propose an approach that can fly a new track in a previously unseen environment without a precise map or expensive data collection.
Our approach represents the global track layout with coarse gate locations, which can be easily estimated from a single demonstration flight.
At test time, a convolutional network predicts the poses of the closest gates along with their uncertainty.
These predictions are incorporated by an extended Kalman filter to maintain optimal maximum-aposteriori estimates of gate locations.
This allows the framework to cope with misleading high-variance estimates that could stem from poor observability or lack of visible gates.
Given the estimated gate poses, we use model predictive control to quickly and accurately navigate through the track.
We conduct extensive experiments in the physical world, demonstrating agile and robust flight through complex and diverse previouslyunseen race tracks.
The presented approach was used to win the IROS 2018 Autonomous Drone Race Competition, outracing the second-placing team by a factor of two.

</details>



# :mortar_board: Education 

* :mortar_board: 2017 - 2021: PhD in Robotics
* :school: 2015 - 2017: MSc in Robotics, Systems, and Control
* :school_satchel: 2011 - 2015: BSc in Mechanical Engineering

# :computer: Coding and Skills
<p align="right">
<img height="24" src="https://cdn.simpleicons.org/git/000/fff" />
<img height="24" src="https://cdn.simpleicons.org/github/000/fff" />
</p>
Most of my contributions now are in private repositories at [Skydio](https://skydio.com), and some open ones at [RPG](https://rpg.ifi.uzh.ch) ([git](https://github.com/uzh-rpg)).

<p align="right">
<img height="24" src="https://cdn.simpleicons.org/cplusplus/000/fff" />
<img height="24" src="https://cdn.simpleicons.org/c/000/fff" />
<img height="24" src="https://cdn.simpleicons.org/python/000/fff" />
<img height="24" src="https://cdn.simpleicons.org/rust/000/fff" />
</p>

I consider myself pretty proficient in C++ and C for anything from real-time embedded systems, over high-performance simulation and modelling, to system architecture design.
I'm also quite familiar with Python for prototyping, experiment design and automation, and data analysis, sprinkled with some SQL.
In my spare time, I love to tinker with Rust, especially the compile-time macro system and some simple embedded applications.

<p align="right">
<img height="24" src="https://cdn.simpleicons.org/neovim/000/fff" />
<img height="24" src="https://cdn.simpleicons.org/llvm/000/fff" />
<img height="24" src="https://cdn.simpleicons.org/bazel/000/fff" />
<img height="24" src="https://cdn.simpleicons.org/pypi/000/fff" />
</p>

To tie everything together, I've worked with various build systems (Bazel, CMake, Ninja) across languages and platforms.
I know a bunch of frameworks (e.g. ROS, LCM, protobuf) and tons of libraries (e.g. Eigen, gtest, catch2, numpy, streamlit, pybind, nanobind).
Most of my work happens in Neovim but sometimes I also use VSCode, usually both in combination with a rich language server and debug setup.
For collaboration, git is all I need (hat tip to [lazygit](https://github.com/jesseduffield/lazygit) and [revup](https://github.com/Skydio/revup),
while for deployment I prefer dockerized builds and a solid CI/CD pipeline (jenkins, actions, buildkyte).

<p align="right">
<img height="24" src="https://cdn.simpleicons.org/numpy/000/fff" />
<img height="24" src="https://cdn.simpleicons.org/streamlit/000/fff" />
<img height="24" src="https://cdn.simpleicons.org/obsidian/000/fff" />
<img height="24" src="https://cdn.simpleicons.org/docker/000/fff" />
</p>

<details>
<summary>
See some more of my technical Skills
</summary>

* Code
  * C++ and C
  * Python
  * Rust

* Configuration
  * Yaml
  * Toml
  * Json

* Documentation
  * Markdown
  * Latex
  * Doxygen & readthedocs

* Workflow Tools
  * Git & Github
  * zsh, tmux, ssh, mosh, atuin
  * lazygit & revup
  * Neovim & VSCode
  * Obsidian & Notion

* Toolchains
  * gcc & clang
  * bazel, cmake, ninja
  * rustc & cargo

* Testing
  * Frameworks
    * gtest, catch2
    * pytest
    * rust test
  * Environments
    * Unit, interface, and integration tests
    * Simulation- and Data-Driven Testing
    * Hardware-in-the-Loop Testing

* Debugging and Profiling
  * gdb
  * lldb across languages
  * valgrind, callgrind
  * kcachegrind, flamegraph

* Deployment
  * Docker
  * Jenkins, Github Actions
  * Prow, Tide

* Embedded
  * Microcontrollers
    * STMicroelectronics STM32
    * Espressif ESP32
    * Atmel AVR
  * System-on-Chip
    * Raspberry Pi, NanoPi, Odroid
    * Nvidia Jetson TX2, Xavier, Nano

* Frameworks
  * ROS and LCM
  * Eigen
  * OpenCV, OpenGL, OpenCL, CUDA
  * numpy, scipy, pandas
  * matplotlib, plotl
  * streamlit
  * pybind11, nanobind

* Data Tools
  * SQL, spark
  * Databricks
  * MongoDB

* Controls
  * System Modelling and Identification
  * Classic PID and adaptive control
  * Linear-Quadratic Methods like LQR and LQG
  * Non-linear Model Predictive Control

* Planning
  * Trajectory Optimization and Time-Optimal Planning
  * Dynamic Programming and Graph Search
  * Sampling-based Planning

* Estimation
  * Kalman Filters
  * Particle Filters
  * Sliding-Window Estimation
  * Global Optimization

* Perception
  * Visual-Inertial Odometry
  * Visual Feature Extraction
  * Stereo and Monocular Depth Estimation

</details>
