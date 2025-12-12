# Task to track where we are on the track and run on the track
import pyb

def TrackRun(shares):
    (s_pos_L, s_pos_R, s_xhat, s_yhat, s_track_section, s_line_follow_en, s_new_setpoint_L, s_new_setpoint_R, s_mot_cmd, s_bump_mask) = shares
   
    # Constants
    r_wheel = 0.035 # [m]
    CPR_L = 5748.5  # [counts per revolution]
    CPR_R = 5748.5  # [counts per revolution]
    w = 0.141       # [m]
    #d1 = 0.75       # Line follow section distance [m]
    d1 = 0.00001
    # d2 = 3.1415*0.2*0.5
    d2 = 0.00002
    d3 = .27 + (3.1415*.15) + .25 +(3.1415*.15*0.5) + (3.1415*.325*0.5) + 0.15 + (3.1415*.225*0.5) + 0.15
    d4 = 0.1*6
    d5 = 0.27
    d6 = 0.65
    d7 = 0.4    # Update
    d8 = 0.5    # Update

    state = 1
    distance = 0
    prev_pos_L = None
    prev_pos_R = None
    base_speed_line = 0.0012
    base_speed_arc = 0.0010

    last_t = pyb.micros()
    
    # Square wave state parameters
    square_wave_stage = None
    square_start_theta = 0
    square_start_distance = 0

    # Grage state parameters
    garage_phase = None
    garage_start_theta = 0

    yield 0

    while True:
        if s_mot_cmd.get() != 1:
            yield 0
            continue

        now_t = pyb.micros()
        dt = ((now_t - last_t) & 0xFFFFFFFF)*(1e-6) # Time difference in seconds
        last_t = now_t
        if dt <= 0.0 or dt > 0.2:   # Safety dt
            dt = 0.02
        
        v_hat = s_xhat.get()
        theta_hat = s_yhat.get()
        distance += v_hat*dt

        # DEBUG: see what TrackRun thinks is happening
        try:
            print("[TR LOOP] dt={:.3f}, v_hat={:+.3f}, dist={:.3f}, state={}".format(
                dt, v_hat, distance, state))
        except Exception:
            pass
        
        # pos_L = s_pos_L.get()
        # pos_R = s_pos_R.get()

        # if prev_pos_L is None:
        #     d_counts_L = 0
        #     d_counts_R = 0
        #     prev_pos_L = pos_L
        #     prev_pos_R = pos_R
        # else:
        #     d_counts_L = pos_L - prev_pos_L
        #     d_counts_R = pos_R - prev_pos_R
        #     prev_pos_L = pos_L
        #     prev_pos_R = pos_R

        #     dsL_m = (2.0*3.1415*r_wheel/CPR_L)*d_counts_L
        #     dsR_m = (2.0*3.1415*r_wheel/CPR_R)*d_counts_R
        #     distance += (dsL_m +dsR_m)*0.5

        # DEBUG EVERY LOOP: what is TrackRun actually seeing?
        try:
            print("[TR LOOP] mot_cmd={:.1f}, state={}, pos_L={}, pos_R={}, dL={}, dR={}, dist={:.5f}".format(
                s_mot_cmd.get(), state, pos_L, pos_R, d_counts_L, d_counts_R, distance))
        except Exception:
            pass
   
        v_hat = s_xhat.get()
        theta_hat = s_yhat.get()

        if state == 1:          # Section 1: line following
            s_track_section.put(1)
            try:
                print("[TrackRun] state={}, distance={:.2f}, square_stage={}".format(
                    state, distance, square_wave_stage))
            except Exception:
                pass
            s_line_follow_en.put(1)
            s_new_setpoint_L.put(0.0012)
            s_new_setpoint_R.put(0.0012)
            if distance >= d1:
                state = 2

        elif state == 2:        # Section 2: choose correct path, do first arc
            s_track_section.put(2)
            try:
                print("[TrackRun] state={}, distance={:.2f}, square_stage={}".format(
                    state, distance, square_wave_stage))
            except Exception:
                pass
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
            try:
                print("[TrackRun] state={}, distance={:.2f}, square_stage={}".format(
                    state, distance, square_wave_stage))
            except Exception:
                pass
            s_line_follow_en.put(1)
            s_new_setpoint_L.put(0.0012)
            s_new_setpoint_R.put(0.0012)
            if distance >= d1 + d2 + d3:
                state = 4
                

        elif state == 4:        # Section 4: square wave
            s_track_section.put(4)
            try:
                print("[TrackRun] state={}, distance={:.2f}, square_stage={}".format(
                    state, distance, square_wave_stage))
            except Exception:
                pass
            s_line_follow_en.put(0)
            straight_distance = 0.1
            
            # do square wave function
            if square_wave_stage is None:
                square_wave_stage = 1
                square_start_theta = theta_hat
                square_start_distance = distance
            square_distance = distance - square_start_distance 

            if square_wave_stage == 1:    # Turn right
                s_new_setpoint_L.put(0.001)
                s_new_setpoint_R.put(-0.001)

                if abs(theta_hat - square_start_theta) >= 3.1415/2:
                    square_wave_stage = 2
                    square_start_distance = distance   # Reset distance for next section
            

            elif square_wave_stage == 2:    # Stright
                s_new_setpoint_L.put(0.001)
                s_new_setpoint_R.put(0.001)
                if square_distance >= straight_distance/2:
                    square_wave_stage = 3
                    square_start_theta = theta_hat
                    square_start_distance = distance

            elif square_wave_stage == 3:    # Turn left
                s_new_setpoint_L.put(-0.001)
                s_new_setpoint_R.put(0.001)
                if abs(theta_hat - square_start_theta) >= 3.1415/2:
                    square_wave_stage = 4
                    square_start_distance = distance

            elif square_wave_stage == 4:    # Straight
                s_new_setpoint_L.put(0.001)
                s_new_setpoint_R.put(0.001)
                if square_distance >= straight_distance:
                    square_wave_stage = 5
                    square_start_theta = theta_hat
                    square_start_distance = distance


            elif square_wave_stage == 5:    # Turn left
                s_new_setpoint_L.put(-0.001)
                s_new_setpoint_R.put(0.001)
                if abs(theta_hat - square_start_theta) >= 3.1415/2:
                    square_wave_stage = 6
                    square_start_distance = distance

            elif square_wave_stage == 6:    # Straight
                s_new_setpoint_L.put(0.001)
                s_new_setpoint_R.put(0.001)
                if square_distance >= straight_distance:
                    square_wave_stage = 7
                    square_start_theta = theta_hat
                    square_start_distance = distance

            elif square_wave_stage == 7:    # Turn right
                s_new_setpoint_L.put(0.001)
                s_new_setpoint_R.put(-0.001)
                if abs(theta_hat - square_start_theta) >= 3.1415/2:
                    square_wave_stage = 8
                    square_start_distance = distance   # Reset distance for next section
            
            elif square_wave_stage == 8:    # Straight
                s_new_setpoint_L.put(0.001)
                s_new_setpoint_R.put(0.001)
                if square_distance >= straight_distance:
                    square_wave_stage = 9
                    square_start_theta = theta_hat
                    square_start_distance = distance

            elif square_wave_stage == 9:    # Turn right
                s_new_setpoint_L.put(0.001)
                s_new_setpoint_R.put(-0.001)

                if abs(theta_hat - square_start_theta) >= 3.1415/2:
                    square_wave_stage = 10
                    square_start_distance = distance   # Reset distance for next section
            
            elif square_wave_stage == 10:   # Straight
                s_new_setpoint_L.put(0.001)
                s_new_setpoint_R.put(0.001)
                if square_distance >= straight_distance:
                    square_wave_stage = 11
                    square_start_theta = theta_hat
                    square_start_distance = distance

            elif square_wave_stage == 11:   # Turn left
                s_new_setpoint_L.put(-0.001)
                s_new_setpoint_R.put(0.001)
                if abs(theta_hat - square_start_theta) >= 3.1415/2:
                    square_wave_stage = 12
                    square_start_distance = distance

            elif square_wave_stage == 12:   # Straight              
                s_new_setpoint_L.put(0.001)
                s_new_setpoint_R.put(0.001)
                if square_distance >= straight_distance:
                    square_wave_stage = 13
                    square_start_theta = theta_hat
                    square_start_distance = distance

            elif square_wave_stage == 13:   # Turn left
                s_new_setpoint_L.put(-0.001)
                s_new_setpoint_R.put(0.001)
                if abs(theta_hat - square_start_theta) >= 3.1415/2:
                    square_wave_stage = 14
                    square_start_distance = distance

            elif square_wave_stage == 14:   # Straight
                s_new_setpoint_L.put(0.001)
                s_new_setpoint_R.put(0.001)
                if square_distance >= straight_distance/2:
                    square_wave_stage = 15
                    square_start_theta = theta_hat
                    square_start_distance = distance

            elif square_wave_stage == 15:   # Turn right
                s_new_setpoint_L.put(0.001)
                s_new_setpoint_R.put(-0.001)
                if abs(theta_hat - square_start_theta) >= 3.1415/2:
                    square_wave_stage = 16
                    square_start_distance = distance   # Reset distance for next section
            
            elif square_wave_stage == 16:
                if distance > d1 + d2 + d3 + d4:
                    state = 5

        elif state == 5:        # Section 5: line following
            s_track_section.put(5)
            try:
                print("[TrackRun] state={}, distance={:.2f}, square_stage={}".format(
                    state, distance, square_wave_stage))
            except Exception:
                pass

            s_line_follow_en.put(1)
            s_new_setpoint_L.put(0.0012)
            s_new_setpoint_R.put(0.0012)
            if distance > d1 + d2 + d3 + d4 + d5:
                state = 6


        elif state == 6:        # Section 6: garage, includes line following 
            s_track_section.put(6)
            try:
                print("[TrackRun] state={}, distance={:.2f}, square_stage={}".format(
                    state, distance, square_wave_stage))
            except Exception:
                pass
            s_line_follow_en.put(0)
            start_theta = theta_hat
            if garage_phase is None:
                garage_phase = 1
                garage_start_theta = theta_hat
            # go straight and then turn at end of garage
            if garage_phase == 1:
                s_new_setpoint_L.put(0.0012)
                s_new_setpoint_R.put(0.0012)
                if distance > d1 + d2 + d3 + d4 + d5 + d6:
                    garage_phase = 2
                    garage_start_theta = theta_hat
            elif garage_phase == 2:
                    s_new_setpoint_L.put(0.001)
                    s_new_setpoint_R.put(-0.001)
            if abs(theta_hat - start_theta) >= 3.1415/2:
                    state = 7
                    garage_phase = None

        elif state == 7:        # Section 7: line following until wall
            s_track_section.put(7)
            try:
                print("[TrackRun] state={}, distance={:.2f}, square_stage={}".format(
                    state, distance, square_wave_stage))
            except Exception:
                pass
            s_line_follow_en.put(1)
            s_new_setpoint_L.put(0.0012)
            s_new_setpoint_R.put(0.0012)
            # if distance > d1 + d2 + d3 + d4 + d5 + d6 + d7:
            if s_bump_mask.get() != 0:
                state = 8

        elif state == 8:        # Section 8: hit wall, go around, rotate
            s_track_section.put(8)
            try:
                print("[TrackRun] state={}, distance={:.2f}, square_stage={}".format(
                    state, distance, square_wave_stage))
            except Exception:
                pass
            s_line_follow_en.put(0)
            s_new_setpoint_L.put(0.0)
            s_new_setpoint_R.put(0.0)
            
        yield 0