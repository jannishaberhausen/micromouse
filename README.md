# SPEEDY GONZALES - A MICROMOUSE PROJECT
## Don't Call It Micromouse - Call It Megamouse
### Ever wondered what Speedy Gonzales does nowadys?

---

**From Wikipedia:**
Micromouse is an event where small robot mice solve a 16×16 maze. It began in the late 1970s. Events are held worldwide, and are most popular in the UK, U.S., Japan, Singapore, India, South Korea and becoming popular in subcontinent countries such as Sri Lanka.

The maze is made up of a 16×16 grid of cells, each 180 mm square with walls 50 mm high. The mice are completely autonomous robots that must find their way from a predetermined starting position to the central area of the maze unaided. The mouse needs to keep track of where it is, discover walls as it explores, map out the maze and detect when it has reached the goal. Having reached the goal, the mouse will typically perform additional searches of the maze until it has found an optimal route from the start to the finish. Once the optimal route has been found, the mouse will run that route in the shortest possible time.

Competitions and conferences are still run regularly.

----

Watch a video of Speedy Gonzales in action [here](https://youtu.be/TwjPslRqBsg).

### Getting Started

Notice `main.c` starts the T1 timer `startTimer()` and the finite state machine (FSM) `plannerFSM()` which realizes the planning of mouse motions before running in an infinite loop in which the behaviour is controlled by the planning FSM and the motion FSM which in turn is called every 50 ms by a timer interupt in T1 (see `timer.c`).
    
The most important motions (and their respective controllers) are defined in `mouse_motion.c` and the business logic for the path planning is implemented in `pathfinder.c`.
