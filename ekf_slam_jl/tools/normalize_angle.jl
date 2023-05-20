function normalize_angle(phi)

    while (phi > pi)
        phi = phi - 2 * pi
    end
    while (phi < -pi)
        phi = phi + 2 * pi
    end
    return phi
end
