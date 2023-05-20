function drawprobellipse(x, C, alpha)
    # Calculate unscaled half axes
    @show x
    sxx = C[1, 1]
    syy = C[2, 2]
    sxy = C[1, 2]
    a = sqrt(0.5 * (sxx + syy + sqrt((sxx - syy)^2 + 4 * sxy^2)))  # always greater
    b = sqrt(0.5 * (sxx + syy - sqrt((sxx - syy)^2 + 4 * sxy^2)))  # always smaller

    # Remove imaginary parts in case of negative definite C
    if ~isreal(a)
        a = real(a)
    end
    if ~isreal(b)
        b = real(b)
    end

    # Scaling in order to reflect specified probability
    a = a * sqrt(chi2invtable(alpha, 2))
    b = b * sqrt(chi2invtable(alpha, 2))

    # Look where the greater half axis belongs to
    if sxx < syy
        swap = a
        a = b
        b = swap
    end

    # Calculate inclination (numerically stable)
    if sxx != syy
        angle = 0.5 * atan(2 * sxy / (sxx - syy))
    elseif sxy == 0
        angle = 0  # angle doesn't matter
    elseif sxy > 0
        angle = pi / 4
    elseif sxy < 0
        angle = -pi / 4
    end
    x[3] = angle

    # Draw ellipse with specified color
    drawellipse(x, a, b)

end

function drawellipse(x, a, b)
    # Constants
    NPOINTS = 100  # point density or resolution

    # Compose point vector
    ivec = 0:2*pi/NPOINTS:2*pi  # index vector
    p = [a*cos.(ivec); b*sin.(ivec)]  # 2 x n matrix which holds ellipse points

    # Translate and rotate
    xo = x[1]
    yo = x[2]
    angle = x[3]
    R = [cos(angle) -sin(angle); sin(angle) cos(angle)]
    T = [xo; yo] * ones(1, length(ivec))
    p = R * p + T

    # Plot
    plot(p[1, :], p[2, :], linewidth=2)
end
