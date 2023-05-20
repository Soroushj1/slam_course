function read_world(filename)
    # Reads the world definition and returns a structure of landmarks.

    # filename: path of the file to load
    # landmarks: structure containing the parsed information

    # Each landmark contains the following information:
    # - id : id of the landmark
    # - x  : x-coordinate
    # - y  : y-coordinate

    # Examples:
    # - Obtain x-coordinate of the 5-th landmark
    #   landmarks[5].x

    input = open(filename)

    landmarks = []

    while !eof(input)
        line = readline(input)
        data = split(line, ' ')

        landmark = (
            id=parse(Float64, data[1]),
            x=parse(Float64, data[2]),
            y=parse(Float64, data[3])
        )
        push!(landmarks, landmark)
    end

    landmarks = landmarks[2:end]

    close(input)

    return landmarks
end