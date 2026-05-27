"""
    Request

A transportation request pairing one pickup node with one dropoff node.
`max_ride_time` is the maximum time the passenger may spend in the vehicle (L_i).
"""
struct Request
    id            :: Int
    pickup        :: Node
    dropoff       :: Node
    max_ride_time :: Float64
end
