using StatsPlots, Distributions
using LinearAlgebra
using LibSerialPort
using DelimitedFiles
using Plots

using Printf

function parse_numbers(s)
    pieces = split(s, ',', keepempty=false)
    return [parse(Float64, ss) for ss in pieces]
end

function main()
    data = zeros(Float64, 1, 5)

    ports = get_port_list()
    port = "COM7"

    dev = LibSerialPort.open(port, 115200)
    sp_flush(dev, SP_BUF_BOTH)

    histogram(data[:, 1])

    try
        while true
            newline = readline(dev)
            try
                row = parse_numbers(newline)

                if abs(row[1] - data[end, 1]) > 10
                    continue
                end

                data = [data; row']
            catch e
                continue
            end

            dist_data = data[:, 1]
            split_dist = dist_data[end-min(20, size(dist_data)[1] - 1):end]

            dist_mean = mean(split_dist)
            dist_std = std(split_dist)

            filter!(e -> 0 < e < 10, split_dist)

            @printf("Distance %.2f stddev %.4f\n\r", dist_mean, dist_std)
            
            if dist_std != NaN
                pl = plot(histogram(split_dist), xlims=[max(0, dist_mean - 5), dist_mean + 5], ylims=[0, 20])
                plot!(pl, Normal(dist_mean, dist_std), linewidth=2)
            end
            display(pl)
        end
    catch e
        close(dev)
    end
end

main()