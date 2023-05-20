function drawrobot(xvec; type = 2, B = 0.4, L = 0.6)
    # Constants
    DEFT = 2            # default robot type
    DEFB = 0.4          # default robot width in [m], defines y-dir. of {R}
    WT   = 0.03         # wheel thickness in [m]
    DEFL = DEFB+0.2     # default robot length in [m]
    WD   = 0.2          # wheel diameter in [m]
    RR   = WT/2         # wheel roundness radius in [m]
    RRR  = 0.04         # roundness radius for rectangular robots in [m]
    HL   = 0.09         # arrow head length in [m]
    CS   = 0.1          # cross size in [m], showing the {R} origin

    # Input argument check
    if !(nargin in [2, 3, 5])
        error("drawrobot: Wrong number of input arguments")
    end

    x = xvec[1]
    y = xvec[2]
    theta = xvec[3]
    T = [x; y]
    R = [cos(theta) -sin(theta); sin(theta) cos(theta)]

    # Main switch statement
    h = []
    if type == 0
        # Draw origin cross
        p = R * [CS -CS 0 0; 0 0 -CS CS] + T .* ones(1, 4)
        h = plot(p[1, 1:2], p[2, 1:2], p[1, 3:4], p[2, 3:4])

    elseif type == 1
        # Draw wheel pair with axis and arrow
        xlw = [x + B/2 * cos(theta + pi/2), y + B/2 * sin(theta + pi/2), theta]
        h1 = drawrect(xlw, WD, WT, RR, 1)  # left wheel
        xlw = [x - B/2 * cos(theta + pi/2), y - B/2 * sin(theta + pi/2), theta]
        h2 = drawrect(xlw, WD, WT, RR, 1)  # right wheel
        # Draw axis cross with arrow
        p = R * [0 0; -B/2 + WT/2 B/2 - WT/2] + T .* ones(1, 2)
        h3 = plot(p[1, :], p[2, :])
        p = R * [L/2; 0] + T
        h4 = drawarrow(T, p, 1, HL)
        h = vcat(h1, h2, h3, h4)

    elseif type == 2
        # Draw wheel pair with axis and arrow
        xlw = [x + B/2 * cos(theta + pi/2), y + B/2 * sin(theta + pi/2), theta]
        h1 = drawrect(xlw, WD, WT, RR, 1)  # left wheel
        xlw = [x - B/2 * cos(theta + pi/2), y - B/2 * sin(theta + pi/2), theta]
        h2 = drawrect(xlw, WD, WT, RR, 1)  # right wheel
        # Draw axis cross with arrow
        p = R * [0 0; -B/2 + WT/2 B/2 - WT/2] + T .* ones(1, 2)
        h3 = plot(p[1, :], p[2, :])
        p = R * [(B + WT)/2; 0] + T
        h4 = drawarrow(T, p, 1, HL)
        # Draw circular contour
        radius = (B + WT)/2
        h5 = drawellipse(xvec, radius, radius)
        h = vcat(h1, h2, h3, h4, h5)

    elseif type == 3
        # Draw circular contour
        radius = (B + WT)/2
        h1 = drawellipse(xvec, radius, radius)
        # Draw line with orientation theta with length radius
        p = R * [(B + WT)/2; 0] + T
        h2 = plot([T[1] p[1]], [T[2] p[2]], linewidth = 2)
        h = vcat(h1, h2)

    elseif type == 4
        # Draw wheel pair with axis and arrow
        xlw = [x + B/2 * cos(theta + pi/2), y + B/2 * sin(theta + pi/2), theta]
        h1 = drawrect(xlw, WD, WT, RR, 1)  # left wheel
        xlw = [x - B/2 * cos(theta + pi/2), y - B/2 * sin(theta + pi/2), theta]
        h2 = drawrect(xlw, WD, WT, RR, 1)  # right wheel
        # Draw axis cross with arrow
        p = R * [0 0; -B/2 + WT/2 B/2 - WT/2] + T .* ones(1, 2)
        h3 = plot(p[1, :], p[2, :])
        p = R * [L/2; 0] + T
        h4 = drawarrow(T, p, 1, HL)
        # Draw rectangular contour
        h5 = drawrect(xvec, L, B, RRR, 0)
        h = vcat(h1, h2, h3, h4, h5)

    elseif type == 5
        # Draw rectangular contour
        h1 = drawrect(xvec, L, B, RRR, 0)
        # Draw line with orientation theta with length L
        p = R * [L/2; 0] + T
        h2 = plot([T[1] p[1]], [T[2] p[2]], linewidth = 2)
        h = vcat(h1, h2)

    else
        error("drawrobot: Unsupported robot type")
    end

    return h
end
