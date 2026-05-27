"""
    Vehicle

One vehicle in the fleet. The classic Cordeau DARP uses a homogeneous fleet,
but this struct supports heterogeneous extensions.
"""
struct Vehicle
    id                 :: Int
    capacity           :: Int       # Q: maximum simultaneous passenger load
    origin             :: DepotNode
    destination        :: DepotNode
    max_route_duration :: Float64   # T: maximum total route time
end
