# Task to track where we are on the track and run on the track
import pyb

def TrackRun(shares):
    (s_pos_L, s_pos_R, s_shat, s_psihat, s_track_section, s_line_follow_en, s_new_setpoint_L, s_new_setpoint_R, s_mot_cmd, s_bump_mask,) = shares
   
    # Constants
    r_wheel = 0.035 # [m]
    CPR_L = 5748.5  # [counts per revolution]
    CPR_R = 5748.5  # [counts per revolution]
    w = 0.141       # [m]
    d1 = 0.08       # Line follow section distance [m]
    d2 = 0.035
    d3 = 0.30
    d4 = 0.03
    d5 = 0.03
    d6 = 0.65
    d7 = 0.4    # Update
    d8 = 0.5    # Update

    state = 1
    prev_state = None
    distance = 0
    prev_pos_L = None
    prev_pos_R = None
    base_speed_line = 0.0012
    base_speed_arc = 0.0010
    state_start_distance = 0
    theta_initial = None
    state4_initialized = False
    target_heading = None

    last_t = pyb.micros()
    
    # Square wave state parameters
    square_wave_stage = None
    square_start_theta = 0
    square_start_distance = 0

    # Garage state parameters
    garage_phase = None
    garage_start_theta = 0

    while True:
        if s_mot_cmd.get() != 1:
            yield 0
            continue

        now_t = pyb.micros()
        dt = ((now_t - last_t) & 0xFFFFFFFF)*(1e-6) # Time difference in seconds
        last_t = now_t
        if dt <= 0.0 or dt > 0.2:   # Safety dt
            dt = 0.02
        
        s_hat = s_shat.get()
        theta_hat = s_psihat.get()
        distance = s_hat

        if theta_initial is None:
            theta_initial = theta_hat
            print("[TrackRun] Captured initial heading θ0 = {:.3f}".format(theta_initial))

        if state != prev_state:
            state_start_distance = s_hat
            prev_state = state
        distance_since_state = s_hat - state_start_distance


        # DEBUG: see what TrackRun thinks is happening
        # try:
        #     print("[TR LOOP] dt={:.3f}, dist={:.3f}, theta={:.3f}, state={}".format(
        #         dt, distance, theta_hat, state))
        # except Exception:
        #     pass
        
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

        # # DEBUG EVERY LOOP: what is TrackRun actually seeing?
        # try:
        #     print("[TR LOOP] mot_cmd={:.1f}, state={}, pos_L={}, pos_R={}, dL={}, dR={}, dist={:.5f}".format(
        #         s_mot_cmd.get(), state, pos_L, pos_R, d_counts_L, d_counts_R, distance))
        # except Exception:
        #     pass

        if state == 1:          # Section 1: line following
            s_track_section.put(1)
            if prev_state != 1:
                theta_initial = theta_hat  # Save absolute orientation at start of run
                print("[TrackRun] Captured initial heading θ0 = {:.3f}".format(theta_initial))
            try:
                print("[TrackRun] state={}, distance={:.2f}, theta={}".format(
                    state, distance_since_state, theta_hat))
            except Exception:
                pass
            s_line_follow_en.put(1)
            s_new_setpoint_L.put(0.0012)
            s_new_setpoint_R.put(0.0012)
            if distance_since_state >= d1:
                state = 2
            yield 0

        elif state == 2:        # Section 2: choose correct path, do first arc
            s_track_section.put(2)
            try:
                print("[TrackRun] state={}, distance={:.2f}, theta={}".format(
                    state, distance_since_state, theta_hat))
            except Exception:
                pass
            s_line_follow_en.put(0)
            # Go forward radius of 0.2 to the right, 1/4 turn
            vL_cmd = 0.0010*(2.0*0.2+w)/(2.0*0.2)
            vR_cmd = 0.0010*(2.0*0.2-w)/(2.0*0.2)
            s_new_setpoint_L.put(vL_cmd)
            s_new_setpoint_R.put(vR_cmd)
            if distance_since_state >= d2:
                state = 3
            yield 0

        elif state == 3:        # Section 3: line follow
            s_track_section.put(3)
            try:
                print("[TrackRun] state={}, distance={:.2f}, theta={}".format(
                    state, distance_since_state, theta_hat))
            except Exception:
                pass
            s_line_follow_en.put(1)
            s_new_setpoint_L.put(0.0012)
            s_new_setpoint_R.put(0.0012)
            if distance_since_state >= d3:
                state = 4
            yield 0


        elif state == 4:  # Section 4: 180° correction

            s_track_section.put(4)
            s_line_follow_en.put(1)
            try:
                print("[TrackRun] state={}, distance={:.2f}, theta={}".format(
                    state, distance_since_state, theta_hat))
            except Exception:
                pass
            s_new_setpoint_L.put(0.0012)
            s_new_setpoint_R.put(0.0012)
            if distance_since_state >= d3:
                state = 5

            # Initialize once on entry
            #
            # if not state4_initialized:
            #
            #     # Compute target
            #     theta_target = theta_initial + 3.1415
            #     # Normalize to [-pi, pi]
            #     if theta_target > 3.1415:
            #         theta_target -= 2 * 3.1415
            #     elif theta_target < -3.1415:
            #         theta_target += 2 * 3.1415
            #     target_heading = theta_target

            #     # Mark initialization done
            #     state4_initialized = True
            #
            #     # Reset distance-for-state
            #     state_start_distance = distance_since_state
            #
            # # Compute heading error EVERY LOOP
            # heading_error = target_heading - theta_hat
            #
            # # Normalize heading error to [-pi, pi]
            # if heading_error > 3.1415:
            #     heading_error -= 2 * 3.1415
            # elif heading_error < -3.1415:
            #     heading_error += 2 * 3.1415
            #
            # # Turn until aligned
            # if abs(heading_error) > 0.10:  # ±6 degrees
            #     turn_speed = 0.0012
            #     if heading_error > 0:
            #         s_new_setpoint_L.put(-turn_speed)
            #         s_new_setpoint_R.put(turn_speed)
            #     else:
            #         s_new_setpoint_L.put(turn_speed)
            #         s_new_setpoint_R.put(-turn_speed)
            #
            # else:
            #     # heading is correct → drive forward
            #     s_new_setpoint_L.put(0.0012)
            #     s_new_setpoint_R.put(0.0012)
            #     if distance_since_state > d4:
            #         state = 5
            #         state4_initialized = False  # reset for later use
            # yield 0


            # straight_distance = 0.1
            #
            # # do square wave function
            # if square_wave_stage is None:
            #     square_wave_stage = 1
            #     square_start_theta = theta_hat
            #     square_start_distance = distance
            # square_distance = distance - square_start_distance
            #
            # if square_wave_stage == 1:    # Turn right
            #     s_new_setpoint_L.put(0.001)
            #     s_new_setpoint_R.put(-0.001)
            #
            #     if abs(theta_hat - square_start_theta) >= 3.1415/2:
            #         square_wave_stage = 2
            #         square_start_distance = distance   # Reset distance for next section
            #
            #
            # elif square_wave_stage == 2:    # Stright
            #     s_new_setpoint_L.put(0.001)
            #     s_new_setpoint_R.put(0.001)
            #     if square_distance >= straight_distance/2:
            #         square_wave_stage = 3
            #         square_start_theta = theta_hat
            #         square_start_distance = distance
            #
            # elif square_wave_stage == 3:    # Turn left
            #     s_new_setpoint_L.put(-0.001)
            #     s_new_setpoint_R.put(0.001)
            #     if abs(theta_hat - square_start_theta) >= 3.1415/2:
            #         square_wave_stage = 4
            #         square_start_distance = distance
            #
            # elif square_wave_stage == 4:    # Straight
            #     s_new_setpoint_L.put(0.001)
            #     s_new_setpoint_R.put(0.001)
            #     if square_distance >= straight_distance:
            #         square_wave_stage = 5
            #         square_start_theta = theta_hat
            #         square_start_distance = distance
            #
            #
            # elif square_wave_stage == 5:    # Turn left
            #     s_new_setpoint_L.put(-0.001)
            #     s_new_setpoint_R.put(0.001)
            #     if abs(theta_hat - square_start_theta) >= 3.1415/2:
            #         square_wave_stage = 6
            #         square_start_distance = distance
            #
            # elif square_wave_stage == 6:    # Straight
            #     s_new_setpoint_L.put(0.001)
            #     s_new_setpoint_R.put(0.001)
            #     if square_distance >= straight_distance:
            #         square_wave_stage = 7
            #         square_start_theta = theta_hat
            #         square_start_distance = distance
            #
            # elif square_wave_stage == 7:    # Turn right
            #     s_new_setpoint_L.put(0.001)
            #     s_new_setpoint_R.put(-0.001)
            #     if abs(theta_hat - square_start_theta) >= 3.1415/2:
            #         square_wave_stage = 8
            #         square_start_distance = distance   # Reset distance for next section
            #
            # elif square_wave_stage == 8:    # Straight
            #     s_new_setpoint_L.put(0.001)
            #     s_new_setpoint_R.put(0.001)
            #     if square_distance >= straight_distance:
            #         square_wave_stage = 9
            #         square_start_theta = theta_hat
            #         square_start_distance = distance
            #
            # elif square_wave_stage == 9:    # Turn right
            #     s_new_setpoint_L.put(0.001)
            #     s_new_setpoint_R.put(-0.001)
            #
            #     if abs(theta_hat - square_start_theta) >= 3.1415/2:
            #         square_wave_stage = 10
            #         square_start_distance = distance   # Reset distance for next section
            #
            # elif square_wave_stage == 10:   # Straight
            #     s_new_setpoint_L.put(0.001)
            #     s_new_setpoint_R.put(0.001)
            #     if square_distance >= straight_distance:
            #         square_wave_stage = 11
            #         square_start_theta = theta_hat
            #         square_start_distance = distance
            #
            # elif square_wave_stage == 11:   # Turn left
            #     s_new_setpoint_L.put(-0.001)
            #     s_new_setpoint_R.put(0.001)
            #     if abs(theta_hat - square_start_theta) >= 3.1415/2:
            #         square_wave_stage = 12
            #         square_start_distance = distance
            #
            # elif square_wave_stage == 12:   # Straight
            #     s_new_setpoint_L.put(0.001)
            #     s_new_setpoint_R.put(0.001)
            #     if square_distance >= straight_distance:
            #         square_wave_stage = 13
            #         square_start_theta = theta_hat
            #         square_start_distance = distance
            #
            # elif square_wave_stage == 13:   # Turn left
            #     s_new_setpoint_L.put(-0.001)
            #     s_new_setpoint_R.put(0.001)
            #     if abs(theta_hat - square_start_theta) >= 3.1415/2:
            #         square_wave_stage = 14
            #         square_start_distance = distance
            #
            # elif square_wave_stage == 14:   # Straight
            #     s_new_setpoint_L.put(0.001)
            #     s_new_setpoint_R.put(0.001)
            #     if square_distance >= straight_distance/2:
            #         square_wave_stage = 15
            #         square_start_theta = theta_hat
            #         square_start_distance = distance
            #
            # elif square_wave_stage == 15:   # Turn right
            #     s_new_setpoint_L.put(0.001)
            #     s_new_setpoint_R.put(-0.001)
            #     if abs(theta_hat - square_start_theta) >= 3.1415/2:
            #         square_wave_stage = 16
            #         square_start_distance = distance   # Reset distance for next section
            #
            # elif square_wave_stage == 16:


        elif state == 5:        # Section 5: line following
            s_track_section.put(5)
            try:
                print("[TrackRun] state={}, distance={:.2f}, theta={}".format(
                    state, distance_since_state, theta_hat))
            except Exception:
                pass

            s_line_follow_en.put(1)
            s_new_setpoint_L.put(0.0012)
            s_new_setpoint_R.put(0.0012)
            if distance_since_state > d5:
                state = 6
            yield 0


        elif state == 6:        # Section 6: garage, includes line following 
            s_track_section.put(6)
            try:
                print("[TrackRun] state={}, distance={:.2f}, theta={}".format(
                    state, distance_since_state, theta_hat))
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
            yield 0

        elif state == 7:        # Section 7: line following until wall
            s_track_section.put(7)
            try:
                print("[TrackRun] state={}, distance={:.2f}, theta={}".format(
                    state, distance_since_state, theta_hat))
            except Exception:
                pass
            s_line_follow_en.put(1)
            s_new_setpoint_L.put(0.0012)
            s_new_setpoint_R.put(0.0012)
            # if distance > d1 + d2 + d3 + d4 + d5 + d6 + d7:
            if s_bump_mask.get() != 0:
                state = 8
            yield 0

        elif state == 8:        # Section 8: hit wall, go around, rotate
            s_track_section.put(8)
            try:
                print("[TrackRun] state={}, distance={:.2f}, theta={}".format(
                    state, distance_since_state, theta_hat))
            except Exception:
                pass
            s_line_follow_en.put(0)
            s_new_setpoint_L.put(0.0)
            s_new_setpoint_R.put(0.0)
            yield 0
            
        yield 0