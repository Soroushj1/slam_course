
function correction_step(mu, sigma, z, observedLandmarks)
    # Updates the belief, i.e., mu and sigma after observing landmarks, according to the sensor model
    # The employed sensor model measures the range and bearing of a landmark
    # mu: 2N+3 vector representing the state mean.
    # The first 3 components of mu correspond to the current estimate of the robot pose [x; y; theta]
    # The current pose estimate of the landmark with id = j is: [mu[2*j+2]; mu[2*j+3]]
    # sigma: 2N+3 x 2N+3 is the covariance matrix
    # z: struct array containing the landmark observations.
    # Each observation z[i] has an id z[i].id, a range z[i].range, and a bearing z[i].bearing
    # The vector observedLandmarks indicates which landmarks have been observed
    # at some point by the robot.
    # observedLandmarks[j] is false if the landmark with id = j has never been observed before.

    # Number of measurements in this time step
    m = length(z)
    # Number of dimensions to mu
    dim = length(mu)

    # Z: vectorized form of all measurements made in this time step: [range_1; bearing_1; range_2; bearing_2; ...; range_m; bearing_m]
    # ExpectedZ: vectorized form of all expected measurements in the same form.
    # They are initialized here and should be filled out in the for loop below
    Z = zeros(m * 2)
    expectedZ = zeros(m * 2)

    # Iterate over the measurements and compute the H matrix
    # (stacked Jacobian blocks of the measurement function)
    # H will be 2m x 2N+3
    # H = Array{Float64}(undef, 2m, dim)
    H =[]

    for i in 1:m
        # Get the id of the landmark corresponding to the i-th observation
        landmarkId = Int(z[i].id)

        # If the landmark is observed for the first time:
        if !observedLandmarks[landmarkId]
            # TODO: Initialize its pose in mu based on the measurement and the current robot pose:
            mu[2 * landmarkId + 2 : 2 * landmarkId + 3] = mu[1:2] + [z[i].range * cos(z[i].bearing + mu[3]); z[i].range * sin(z[i].bearing + mu[3])]

            # Indicate in the observedLandmarks vector that this landmark has been observed
            observedLandmarks[landmarkId] = true
        end

        # TODO: Add the landmark measurement to the Z vector
        Z[2 * i - 1 : 2 * i] = [z[i].range; z[i].bearing]

        # TODO: Use the current estimate of the landmark pose
        # to compute the corresponding expected measurement in expectedZ:
        delta = mu[2 * landmarkId + 2 : 2 * landmarkId + 3] - mu[1:2]
        q = delta' * delta

        expectedZ[2 * i - 1] = sqrt(q)
        expectedZ[2 * i] = normalize_angle(atan(delta[2], delta[1]) - mu[3])  

        # TODO: Compute the Jacobian Hi of the measurement function h for this observation
        Hi = 1 / q * [-sqrt(q)*delta[1] -sqrt(q)*delta[2] 0 sqrt(q)*delta[1] sqrt(q)*delta[2]; delta[2] -1*delta[1] -q -1* delta[2] delta[1]]

        # Map Jacobian Hi to high dimensional space by a mapping matrix Fxj
        Fxj = zeros(5, dim)
        Fxj[1:3, 1:3] = I(3)
        Fxj[4, 2 * landmarkId + 2] = 1
        Fxj[5, 2 * landmarkId + 3] = 1

        Hi = Hi * Fxj

        # Augment H with the new Hi
        H = [H; Hi]
    end

    # TODO: Construct the sensor noise matrix Q

    Q= (0.01*I)(2 * m)

    # TODO: Compute the Kalman gain
    K = sigma * H' * inv(H * sigma * H' + Q)

    # TODO: Compute the difference between the expected and recorded measurements.
    # Remember to normalize the bearings after subtracting!
    # (hint: use the normalize_all_bearings function available in tools)
    diffZ = normalize_all_bearings.(Z - expectedZ)

    # TODO: Finish the correction step by computing the new mu and sigma.
    # Normalize theta in the robot pose.
    mu += K * diffZ
    sigma = (Matrix{Float64}(I, dim, dim) - K * H) * sigma

    return mu, sigma, observedLandmarks
end

#
