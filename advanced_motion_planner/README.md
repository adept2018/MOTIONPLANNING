Advanced motion planner which selects the best available path(s). Each path is an rectangle suitable for passage of the car.
Developer: Alexander Konovalenko
Latest comments:
* 2019-06-19  Demo has been performed on 190618. This is final commit to AMP.
*            Things left for further improvements:
*         1) extend decision making in motion_computer.cpp lines# 128-132,
*          as Chrais pointed out, ratio r_best/w_best can be used with proper
*          range (e.g. 0.8...1.2) check as additional condition on selection
*          of the best path in lines# 148-170 of motion_computer.cpp
*         2) Alan's implementation of reading parameters from a file and
*          their real-time update shall be used instead of many macro defines
*          in amp_common.h
*         3) Implemented backward motion shall be further debugged and improved
*          (it is disabled right now in amp_common.h).
*         4) Positive noise (appearing ghost points) filtering is disabled
*          (macro MAX_ALLOWED_POINTS) because of bigger problem with missing
*          points (negative noise).
*         5) BETTER LIDAR IS WANTED!!!
