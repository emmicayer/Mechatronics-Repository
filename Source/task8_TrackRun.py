<<<<<<< Updated upstream
# Task to track where we are on the track and run on the track
import pyb

def TrackRun(shares):
    (s_pos_L, s_pos_R, s_xhat, s_yhat, s_track_section, s_line_follow_en, s_new_setpoint_L, s_new_setpoint_R, s_mot_cmd) = shares
    state = 1 
    distance = 0
   
    # Constants
    r_wheel = 0.035 # [m]
    CPR_L = 5748.5  # [counts per revolution]
    CPR_R = 5748.5  # [counts per revolution]
    w = 0.141       # [m]
    d1 = 0.75       # Line follow section distance [m]
    d2 = 3.1415*0.2*0.5
    d3 = .27 + (3.1415*.15) + .25 +(3.1415*.15*0.5) + (3.1415*.325*0.5) + 0.15 + (3.1415*.225*0.5) + 0.15
    d4 = 0.1*6
    d5 = 
    d6 = 
    d7 = 
    d8 = 

    state = 1
    square_wave_stage = None
    distance = 0
    prev_pos_L = None
    prev_pos_R = None
    base_speed_line = 0.0012
    base_speed_arc = 0.0010

    yield 0

    while True:
        if s_mot_cmd.get() != 1.0:
            yield 0
            continue
        pos_L = s_pos_L.get()
        pos_R = s_pos_R.get()
        if prev_pos_L == None:
            prev_pos_L = pos_L
            prev_pos_R = pos_R
        else:
            d_counts_L = pos_L - prev_pos_L
            d_counts_R = pos_R - prev_pos_R
            prev_pos_L = pos_L
            prev_pos_R = pos_R

            dsL_m = (2.0*3.1415*r_wheel/CPR_L)*d_counts_L
            dsR_m = (2.0*3.1415*r_wheel/CPR_R)*d_counts_R
            distance = (dsL_m +dsR_m)*0.5
        v_hat = s_xhat.get()
        theta_hat = s_yhat.get()


        if state == 1:          # Section 1: line following
            s_track_section.put(1)
            s_line_follow_en.put(1)
            s_new_setpoint_L.put(0.0012)
            s_new_setpoint_R.put(0.0012)
            if distance >= d1:
                state = 2

        elif state == 2:        # Section 2: choose correct path, do first arc
            s_track_section.put(2)
            s_line_follow_en.put(0)
            # Go forward radius of 0.2 to the right, 1/4 turn
            vL_cmd = 0.0010*(2.0*0.2-w)/(2.0*0.2)
            vR_cmd = 0.0010*(2.0*0.2+w)/(2.0*0.2)
            s_new_setpoint_L.put(vL_cmd)
            s_new_setpoint_R.put(vR_cmd)
            if distance >= d1 + d2:
                state = 3

        elif state == 3:        # Section 3: line follow
            s_track_section.put(3)
            s_line_follow_en.put(1)
            s_new_setpoint_L.put(0.0012)
            s_new_setpoint_R.put(0.0012)
            if distance >= d1 + d2 + d3:
                state = 4
                

        elif state == 4:        # Section 4: square wave
            s_track_section.put(4)
            s_line_follow_en.put(0)
            straight_distance = 0.1
            
            # do square wave function
            if square_wave_stage is None:
                square_wave_stage = 1
                square_start_theta = theta_hat
                square_start_distance = distance
            
            elif square_wave_stage == 1:    # Turn right
                s_new_setpoint_L.put(0.001)
                s_new_setpoint_R.put(-0.001)
                if abs(theta_hat - square_start_theta) >= 3.1415/2:
                    square_wave_stage = 2
                    square_start_distance = distance   # Reset distance for next section


            elif square_wave_stage == 2:    # Stright
                s_new_setpoint_L.put(0.001)
                s_new_setpoint_R.put(0.001)
                if (straight_distance - square_start_distance) >= 0.1



            elif square_wave_stage == 3:



            if distance > d1 + d2 + d3 + d4:
                state = 5








        elif state == 5:        # Section 5: line following
            s_track_section.put(5)
            s_line_follow_en.put(1)
            s_new_setpoint_L.put(0.0012)
            s_new_setpoint_R.put(0.0012)
            if distance > d1 + d2 + d3 + d4 + d5:
                state = 6


        elif state == 6:        # Section 6: garage
            s_track_section.put(6)
            s_line_follow_en.put(0)
            # go straight and then turn at end of garage
            if distance > d1 + d2 + d3 + d4 + d5:
                state = 7


        elif state == 7:        # Section 7: line following until wall
            s_track_section.put(7)
            s_line_follow_en.put(1)
            s_new_setpoint_L.put(0.0012)
            s_new_setpoint_R.put(0.0012)
            if distance > d1 + d2 + d3 + d4 + d5 + d6 + d7:
                state = 8


        elif state == 8:        # Section 8: hit wall, go around, rotate
            s_track_section.put(8)
            s_line_follow_en.put(0)
            
        yield 0

=======
# Task to track where we are on the track and run on the track
import pyb

def TrackRun(shares):
    (s_pos_L, s_pos_R, s_xhat, s_yhat, s_track_section, s_line_follow_en, s_new_setpoint_L, s_new_setpoint_R, s_mot_cmd) = shares
    state = 1 
    distance = 0
   
    # Constants
    r_wheel = 0.035 # [m]
    CPR_L = 5748.5  # [counts per revolution]
    CPR_R = 5748.5  # [counts per revolution]
    w = 0.141       # [m]
    d1 = 0.75       # Line follow section distance [m]
    d2 = 3.1415*0.2*0.5
    d3 = .27 + (3.1415*.15) + .25 +(3.1415*.15*0.5) + (3.1415*.325*0.5) + 0.15 + (3.1415*.225*0.5) + 0.15
    d4 = 0.1*6
    d5 = 
    d6 = 
    d7 = 
    d8 = 

    state = 1
    square_wave_stage = None
    distance = 0
    prev_pos_L = None
    prev_pos_R = None
    base_speed_line = 0.0012
    base_speed_arc = 0.0010

    yield 0

    while True:
        if s_mot_cmd.get() != 1.0:
            yield 0
            continue
        pos_L = s_pos_L.get()
        pos_R = s_pos_R.get()
        if prev_pos_L == None:
            prev_pos_L = pos_L
            prev_pos_R = pos_R
        else:
            d_counts_L = pos_L - prev_pos_L
            d_counts_R = pos_R - prev_pos_R
            prev_pos_L = pos_L
            prev_pos_R = pos_R

            dsL_m = (2.0*3.1415*r_wheel/CPR_L)*d_counts_L
            dsR_m = (2.0*3.1415*r_wheel/CPR_R)*d_counts_R
            distance = (dsL_m +dsR_m)*0.5
        v_hat = s_xhat.get()
        theta_hat = s_yhat.get()


        if state == 1:          # Section 1: line following
            s_track_section.put(1)
            s_line_follow_en.put(1)
            s_new_setpoint_L.put(0.0012)
            s_new_setpoint_R.put(0.0012)
            if distance >= d1:
                state = 2

        elif state == 2:        # Section 2: choose correct path, do first arc
            s_track_section.put(2)
            s_line_follow_en.put(0)
            # Go forward radius of 0.2 to the right, 1/4 turn
            vL_cmd = 0.0010*(2.0*0.2-w)/(2.0*0.2)
            vR_cmd = 0.0010*(2.0*0.2+w)/(2.0*0.2)
            s_new_setpoint_L.put(vL_cmd)
            s_new_setpoint_R.put(vR_cmd)
            if distance >= d1 + d2:
                state = 3

        elif state == 3:        # Section 3: line follow
            s_track_section.put(3)
            s_line_follow_en.put(1)
            s_new_setpoint_L.put(0.0012)
            s_new_setpoint_R.put(0.0012)
            if distance >= d1 + d2 + d3:
                state = 4
                

        elif state == 4:        # Section 4: square wave
            s_track_section.put(4)
            s_line_follow_en.put(0)
            straight_distance = 0.1
            
            # do square wave function
            if square_wave_stage is None:
                square_wave_stage = 1
                square_start_theta = theta_hat
                square_start_distance = distance
            
            elif square_wave_stage == 1:    # Turn right
                s_new_setpoint_L.put(0.001)
                s_new_setpoint_R.put(-0.001)
                if abs(theta_hat - square_start_theta) >= 3.1415/2:
                    square_wave_stage = 2
                    square_start_distance = distance   # Reset distance for next section


            elif square_wave_stage == 2:    # Stright
                s_new_setpoint_L.put(0.001)
                s_new_setpoint_R.put(0.001)
                if (straight_distance - square_start_distance) >= 0.1



            elif square_wave_stage == 3:



            if distance > d1 + d2 + d3 + d4:
                state = 5








        elif state == 5:        # Section 5: line following
            s_track_section.put(5)
            s_line_follow_en.put(1)
            s_new_setpoint_L.put(0.0012)
            s_new_setpoint_R.put(0.0012)
            if distance > d1 + d2 + d3 + d4 + d5:
                state = 6


        elif state == 6:        # Section 6: garage
            s_track_section.put(6)
            s_line_follow_en.put(0)
            # go straight and then turn at end of garage
            if distance > d1 + d2 + d3 + d4 + d5:
                state = 7


        elif state == 7:        # Section 7: line following until wall
            s_track_section.put(7)
            s_line_follow_en.put(1)
            s_new_setpoint_L.put(0.0012)
            s_new_setpoint_R.put(0.0012)
            if distance > d1 + d2 + d3 + d4 + d5 + d6 + d7:
                state = 8


        elif state == 8:        # Section 8: hit wall, go around, rotate
            s_track_section.put(8)
            s_line_follow_en.put(0)
            
        yield 0

>>>>>>> Stashed changes
