#=
main:
- Julia version: 1.8.5
- Author: Geby
- Date: 2023-02-01
=#
struct wheels
    radius::Float64
    friction::Float64
    rpm::Int64
end
mutable struct robot 
    size::Float64
    x_pos::Float64
    y_pos::Float64
    speed::Float64
end
function sim(revs::Float64,rpm::Int64,circumfrence::Float64)
    i=0
    while i<= (revs)
        println("revs: $revs, distance: $((circumfrence)*i)")
        println("t = $(i/revs)")

        # println("pulse: $i,rev: $(i/1000), t= $((i/1000)/rpm)m")
        # println("distance: $((i/1000)*(2*pi*(wheel_radius/12)))")
        i+=0.5
    end
    to_station = 8.166

end
mec_wheels = wheels(3,0,10)

bob = robot(2.67,9.25,2.67)


distance_ft = 18.667-bob.size
circ = (2*pi*(wheel_radius/12))

revolutions =((distance_ft)/circ)

sim(revolutions, rpm,circ)
