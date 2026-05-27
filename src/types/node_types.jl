"""
Abstract supertype for all nodes in a DARP instance.
"""
abstract type AbstractNode end

"""
    Node

A pickup or dropoff stop. Sign of `load` distinguishes role:
positive for pickups, negative for dropoffs (Cordeau convention).
"""
struct Node <: AbstractNode
    id           :: Int
    x            :: Float64
    y            :: Float64
    service_time :: Float64   # d_i: service duration
    tw_start     :: Float64   # e_i: earliest service begin
    tw_end       :: Float64   # l_i: latest service begin
    load         :: Int       # q_i: +q for pickup, -q for dropoff
end

const Station = Node

"""
    DepotNode

Origin or destination depot. No load demand.
"""
struct DepotNode <: AbstractNode
    id           :: Int
    x            :: Float64
    y            :: Float64
    service_time :: Float64
    tw_start     :: Float64
    tw_end       :: Float64
end
