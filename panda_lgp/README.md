1. vseobecna class-a KOMO pre 3. a 2. level
- mal by som byt schopny pouzit rovnake constraints pre oba levely
- nastavit moznost "phase" -> ktore constraints platia pre rozlicne fazy


2. action skeleton
- asi vo forme struct kde bude zadefinovane:
    - zoznam akcii
    - kazda akcia musi byt samostatne zadefinovana
        - ake constraints reprezentuje


panda_lgp 
    - include
    - src
        - constraints -> Position, Orientation, Distance, Grasp, ...
        - KOMO -> objective function, funkcie na pridanie constraints, urcenie pre ktoru fazu patria dane constraints, NLOPT/IFOPT bridge
        - utils -> DH matrices, geometric Jacobian, FK, marker visualization, ...
        - action_skeleton -> define actions
        - main.cpp:
            1. Hand write the action sequence (simple array with names maybe) -> not very important without TP
            2. Initialize joint limits, KOMO (number of phases = number of actions, number of timesteps within the actions), add constraints to phases, define initial facts
            3. Solve KOMO for phases (2. level)
            4. If the solution has been found, solve for timesteps within the phases (update start and goal configurations according to the second level)
            5. Visualize in Rviz
            6. Execute the whole plan
        
        - Logic -> TODO: not important for now, logic expressions, FoL, maybe tree





