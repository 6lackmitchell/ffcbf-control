from simulate import simulate

vehicle = 'bicycle'
level = 'dynamic'
# situation = 'intersection'
# situation = 'warehouse'
situation = 'dasclab'

if __name__ == "__main__":
    end_time = 40.0
    timestep = 0.05

    # while True:

    success = simulate(end_time, timestep, vehicle, level, situation)
        # if success:
        #
        #     break
