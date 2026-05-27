using DelimitedFiles

"""
    read_instance(filepath; name="") :: DARPInstance

Parse a Cordeau-format DARP instance file.

File layout:
  Line 1:      K  n  T  Q  L          (vehicles, requests, max duration, capacity, max ride time)
  Line 2:      0  x  y  d  e  l  q   (origin depot)
  Lines 3..n+2:    pickup nodes 1..n
  Lines n+3..2n+2: dropoff nodes n+1..2n
  Line 2n+3:   return depot node

If `name` is empty it is inferred from the filename without extension.
"""
function read_instance(filepath::String; name::String="") :: DARPInstance
    data = readdlm(filepath)

    K     = Int(data[1, 1])
    n     = Int(data[1, 2])
    T     = Float64(data[1, 3])
    Q     = Int(data[1, 4])
    L     = Float64(data[1, 5])

    depot_origin      = _parse_depot(data[2, :])
    pickup_nodes      = [_parse_node(data[i + 2, :]) for i in 1:n]
    dropoff_nodes     = [_parse_node(data[n + 2 + i, :]) for i in 1:n]
    depot_destination = _parse_depot(data[2*n + 3, :])

    requests = [Request(i, pickup_nodes[i], dropoff_nodes[i], L) for i in 1:n]
    nodes    = vcat(pickup_nodes, dropoff_nodes)

    # Build N×N coordinate matrix for travel matrices
    # Julia indices: 1=origin, 2..n+1=pickups, n+2..2n+1=dropoffs, 2n+2=destination
    N_total = 2*n + 2
    coords  = zeros(Float64, N_total, 2)
    coords[1, :]       = [depot_origin.x, depot_origin.y]
    for i in 1:n
        coords[i+1, :]   = [pickup_nodes[i].x, pickup_nodes[i].y]
        coords[n+i+1, :] = [dropoff_nodes[i].x, dropoff_nodes[i].y]
    end
    coords[N_total, :] = [depot_destination.x, depot_destination.y]

    travel_dist, travel_time = _build_travel_matrices(coords)

    vehicles = [Vehicle(k, Q, depot_origin, depot_destination, T) for k in 1:K]

    inst_name = isempty(name) ? splitext(basename(filepath))[1] : name

    return DARPInstance(
        inst_name, n, K, Q, T, L,
        depot_origin, depot_destination,
        requests, nodes, vehicles,
        travel_time, travel_dist
    )
end

function _parse_node(row::AbstractVector) :: Node
    return Node(
        Int(row[1]),
        Float64(row[2]),
        Float64(row[3]),
        Float64(row[4]),
        Float64(row[5]),
        Float64(row[6]),
        Int(row[7])
    )
end

function _parse_depot(row::AbstractVector) :: DepotNode
    return DepotNode(
        Int(row[1]),
        Float64(row[2]),
        Float64(row[3]),
        Float64(row[4]),
        Float64(row[5]),
        Float64(row[6])
    )
end

function _build_travel_matrices(coords::Matrix{Float64})
    N = size(coords, 1)
    dist = zeros(Float64, N, N)
    for i in 1:N, j in 1:N
        if i != j
            dx = coords[i, 1] - coords[j, 1]
            dy = coords[i, 2] - coords[j, 2]
            dist[i, j] = sqrt(dx*dx + dy*dy)
        end
    end
    return dist, copy(dist)   # (travel_distance, travel_time) — unit speed assumed
end
