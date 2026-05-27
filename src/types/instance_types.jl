"""
    DARPInstance

Complete problem data for one DARP instance.

Travel matrices are (2n+2)×(2n+2) with Julia 1-based indexing:
  index 1      = origin depot  (Cordeau node 0)
  index 2..n+1 = pickups       (Cordeau nodes 1..n)
  index n+2..2n+1 = dropoffs   (Cordeau nodes n+1..2n)
  index 2n+2   = return depot  (Cordeau node 2n+1)

`nodes` stores 2n Node objects: pickups first (indices 1..n), then dropoffs (n+1..2n).
"""
struct DARPInstance
    name              :: String
    n                 :: Int       # number of requests
    K                 :: Int       # number of vehicles
    Q                 :: Int       # vehicle capacity
    T                 :: Float64   # max route duration
    L                 :: Float64   # max ride time (uniform)
    depot_origin      :: DepotNode
    depot_destination :: DepotNode
    requests          :: Vector{Request}
    nodes             :: Vector{Node}     # [pickups 1..n, dropoffs n+1..2n]
    vehicles          :: Vector{Vehicle}
    travel_time       :: Matrix{Float64}  # (2n+2)×(2n+2)
    travel_distance   :: Matrix{Float64}  # (2n+2)×(2n+2), Euclidean
end
