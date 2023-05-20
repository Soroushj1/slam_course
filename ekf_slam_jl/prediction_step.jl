function prediction_step(mu, sigma, u)
    # Updates the belief concerning the robot pose according to the motion model,
    # mu: 2N+3 x 1 vector representing the state mean
    # sigma: 2N+3 x 2N+3 covariance matrix
    # u: odometry reading (r1, t, r2)
    # Use u.r1, u.t, and u.r2 to access the rotation and translation values

    # TODO: Compute the new mu based on the noise-free (odometry-based) motion model
    # Remember to normalize theta after the update (hint: use the function normalize_angle available in tools)

    # mu_vec = zeros(length(mu))

    # mu_vec[1] += u.t * cos(mu[3] + u.r1)
    # mu_vec[2] += u.t * sin(mu[3] + u.r1)
    # mu_vec[3] += u.r1 + u.r2
    # mu_vec[3] = normalize_angle(mu_vec[3])

    mu_vec = motion_model(mu,u)
    G = ForwardDiff.jacobian(_mu -> motion_model(_mu,u), mu)
    # The jacobian is largly correct compared to the solution but two elements need to be swapped... Don't know the reason for this behavior

    # # Compute the 3x3 Jacobian Gx of the motion model by hand as oppose to using ForwardDiff
    # Gx = Matrix{Float64}(I, 3, 3)
    # Gx[1, 3] = -u.t * cos(mu[3] + u.r1)
    # Gx[2, 3] = u.t * sin(mu[3] + u.r1)

    # # Construct the full Jacobian G
    # G = [Gx zeros(3, length(mu) - 3); zeros(length(mu) - 3, 3) Matrix{Float64}(I, length(mu) - 3, length(mu) - 3)]

    # Motion noise
    motionNoise = 0.1
    R3 = Diagonal([motionNoise,motionNoise, motionNoise/10])
    R = zeros(size(sigma, 1), size(sigma, 2))
    R[1:3, 1:3] = R3

    # Compute the predicted sigma after incorporating the motion
    sigma = G * sigma * G' + R

    return mu_vec, sigma
end


function motion_model(mu, u)
    
    Fx = [I(3) zeros(3, length(mu)-3)]
    return mu + Fx'*[u.t * cos(mu[3] + u.r1); u.t * sin(mu[3] + u.r1); normalize_angle(u.r1 + u.r2)]

end
