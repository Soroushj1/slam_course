function plot_state(mu, sigma, landmarks, timestep, observedLandmarks, z, window)
    # Visualizes the state of the EKF SLAM algorithm.
    #
    # The resulting plot displays the following information:
    # - map ground truth (black +'s)
    # - current robot pose estimate (red)
    # - current landmark pose estimates (blue)
    # - visualization of the observations made at this time step (line between robot and landmark)

    L = [landmarks[key] for key in keys(landmarks)]
    # drawprobellipse(mu[1:3], sigma[1:3, 1:3], 0.6)
    scatter!([l[1] for l in L], [l[2] for l in L], markershape=:cross, markersize=10, linecolor=:black, linewidth=5)

    for i in eachindex(observedLandmarks)
        if observedLandmarks[i]
            scatter!([mu[2*i+2]], [mu[2*i+3]], markershape=:circle, markersize=10, linecolor=:blue, linewidth=5)
            # drawprobellipse(mu[2*i+2:2*i+3], sigma[2*i+2:2*i+3, 2*i+2:2*i+3], 0.6)
        end
    end

    for i in eachindex(z)
        mX = mu[2*Int(z[i].id)+2]
        mY = mu[2*Int(z[i].id)+3]
        plot!([mu[1], mX], [mu[2], mY], linecolor=:black, linewidth=1)
    end

    scatter!([mu[1]], [mu[2]], markershape=:circle, markersize=10, linecolor=:red, linewidth=5)
    plot!([mu[1]], [mu[2]], seriestype=:scatter, markershape=:rect, markersize=30, linecolor=:red, linewidth=5)

    # xlims!(-2, 12)
    # ylims!(-2, 12)

    if window
        display(plot!())
    else
        filename = "plots/ekf_$(string(timestep, pad=3, base=10)).png"
        savefig(filename)
    end
end