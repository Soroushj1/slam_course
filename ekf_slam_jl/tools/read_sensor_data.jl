function read_data(filename)
    # Reads the odometry and sensor readings from a file.

    # filename: path to the file to parse
    # data: structure containing the parsed information

    # The data is returned in a structure where the u_t and z_t are stored
    # within a single entry. A z_t can contain observations of multiple
    # landmarks.

    # Usage:
    # - access the readings for timestep i:
    #   data.timestep(i)
    #   this returns a structure containing the odometry reading and all
    #   landmark obsevations, which can be accessed as follows
    # - odometry reading at timestep i:
    #   data.timestep(i).odometry
    # - senor reading at timestep i:
    #   data.timestep(i).sensor

    # Odometry readings have the following fields:
    # - r1 : rotation 1
    # - t  : translation
    # - r2 : rotation 2
    # which correspond to the identically labeled variables in the motion
    # mode.

    # Sensor readings can again be indexed and each of the entris has the
    # following fields:
    # - id      : id of the observed landmark
    # - range   : measured range to the landmark
    # - bearing : measured angle to the landmark (you can ignore this)

    # Examples:
    # - Translational component of the odometry reading at timestep 10
    #   data.timestep(10).odometry.t
    # - Measured range to the second landmark observed at timestep 4
    #   data.timestep(4).sensor(2).range

    input = open(filename)

    data = (
        timestep = [],
    )
    first = true

    odom = ()
    sensor = []

    while !eof(input)
        line = readline(input)
        arr = split(line, ' ')
        type = strip(arr[1])

        if type == "ODOMETRY"
            if !first
                push!(data.timestep, (
                    odometry = odom,
                    sensor = sensor[2:end],
                ))
                odom = ()
                sensor = []
            end
            first = false
            odom = (
                r1 = parse(Float64, arr[2]),
                t = parse(Float64, arr[3]),
                r2 = parse(Float64, arr[4]),
            )
        elseif type == "SENSOR"
            reading = (
                id = parse(Float64, arr[2]),
                range = parse(Float64, arr[3]),
                bearing = parse(Float64, arr[4]),
            )
            push!(sensor, reading)
        end
    end

    # data.timestep = data.timestep[2:end]

    close(input)

    return data
end
