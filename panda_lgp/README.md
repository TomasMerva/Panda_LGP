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
        - utils -> DH matrices, geometric Jacobian, FK, marker visualization, ...
        - KOMO -> objective function, funkcie na pridanie constraints, urcenie pre ktoru fazu patria dane constraints, NLOPT/IFOPT bridge
               -> mohlo by to byt tak, ze automaticky vytvori premenne


        - constraints -> Position, Orientation, Distance, Grasp, ...
        - action_skeleton -> define actions
        - main.cpp:
            1. Hand write the action sequence (simple array with names maybe) -> not very important without TP
            2. Initialize joint limits, KOMO (number of phases = number of actions, number of timesteps within the actions), add constraints to phases, define initial facts
            3. Solve KOMO for phases (2. level)
            4. If the solution has been found, solve for timesteps within the phases (update start and goal configurations according to the second level)
            5. Visualize in Rviz
            6. Execute the whole plan
        
        - Logic -> TODO: not important for now, logic expressions, FoL, maybe tree




1. Potrebujem dajak priradit nazov framu, ku danemu objektu? Alebo presnejsie musim vediet, ze o ktory objekt sa jedna na zaklade nazvu
2. Ked budem mat toto tak potom by som mal vediet vyjadrit constraints
- teoreticky to viem urobit naopak, vyjadrit constraints a hodnoty dat na zaklade nazvov
3. Vedel by som ukladat smerniky na funkciu pre constraints s tym, ze menili by sa iba Data -> cize zrobit nejaku general strukturu pre constraints
Problem je ze sa mi mozno menia dimenzia decision variables ale asi srat na to

Co mam zatial:
    1. priradenu geometricku constraint pre phase
    2. priradene data pre constraint
    3. boundaries for decision variables
    4. Init state potrebujem dat


Po kade som dosiel
- 12.10.
    - napriek tomu ze mam tam tu constraint tak to nejde pre druhu fazu
    - skus mozno vygenerovat init guess z daneho regionu