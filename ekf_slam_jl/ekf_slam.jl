# This is the main extended Kalman filter SLAM loop. This script calls all the required
# functions in the correct order.
#
# You can disable the plotting or change the number of steps the filter
# runs for to ease the debugging. You should, however, not change the order
# or calls of any of the other lines, as it might break the framework.
#
# If you are unsure about the input and return values of functions, you
# should read their documentation, which tells you the expected dimensions.

let
    import Pkg
    Pkg.add("LinearAlgebra")
    Pkg.add("ForwardDiff")
    Pkg.add("Plots")
    Pkg.add("Distributions")

    using LinearAlgebra, ForwardDiff, Plots, Distributions

    # Read world data, i.e., landmarks. The true landmark positions are not given to the robot
    @show tools = joinpath(@__DIR__, "tools/")
    include.(readdir(tools; join=true))
    include(joinpath(@__DIR__, "prediction_step.jl"))
    include(joinpath(@__DIR__, "correction_step.jl"))

    landmarks = read_world("data/world.dat")
    data = read_data("data/sensor_data.dat")

    INF = 1000

    # Get the number of landmarks in the map
    N = size(landmarks, 1)

    # observedLandmarks is a vector that keeps track of which landmarks have been observed so far.
    # observedLandmarks[i] will be true if the landmark with id = i has been observed at some point by the robot
    observedLandmarks = falses(N)

    # Initialize belief:
    # mu: 2N+3x1 vector representing the mean of the normal distribution
    # The first 3 components of mu correspond to the pose of the robot,
    # and the landmark poses (xi, yi) are stacked in ascending id order.
    # sigma: (2N+3)x(2N+3) covariance matrix of the normal distribution

    mu = zeros(2 * N + 3)
    robSigma = zeros(3, 3)
    robMapSigma = zeros(3, 2 * N)
    mapSigma = INF * I(2 * N)
    sigma = [robSigma robMapSigma; robMapSigma' mapSigma]

    frames = []

    # Perform filter update for each odometry-observation pair read from the data file.
    for t in 1:size(data.timestep, 1)
        # Perform the prediction step of the EKF
        mu_new, sigma_new = prediction_step(mu, sigma, data.timestep[t].odometry)

        # Perform the correction step
        mu, sigma, observedLandmarks = correction_step(mu_new, sigma_new, data.timestep[t].sensor, observedLandmarks)
        plot()
        # Generate visualization plots of the current state of the filter
        plot_state(mu, sigma, landmarks, t, observedLandmarks, data.timestep[t].sensor, false)

        println("Current state vector:")
        println("mu = ", mu)
        print("timestep: $t")
    end

    # Display the final system covariance matrix
    println("Final system covariance matrix:")
    println(sigma)
    # Display the final state estimate
    println("Final robot pose:")
    println("mu_robot = ", mu[1:3])
    println("sigma_robot = ")
    println(sigma[1:3, 1:3])
    
end



