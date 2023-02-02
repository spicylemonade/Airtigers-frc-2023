#=
main:
- Julia version: 1.8.5
- Author: Geby
- Date: 2023-02-01
=#
function sim(revs::Float64,pulse::Int64, rpm::Int64)
    i=0
    while i<= (revs*pulse)
        println("pulse: $i,rev: $(i/1000), t= $((i/1000)/rpm)m")
        println("distance: $((i/1000)*(2*pi*(wheel_radius/12)))")
        i+=1
    end
end
    
rpm = 10
#(rpm)x = total revs
    
wheel_radius = 3
#in inches
pulses_per_rev = 1000 #encoder pulses

distance_ft = 18.667

revolutions =(distance_ft)/(2*pi*(wheel_radius/12))

sim(revolutions, pulses_per_rev, rpm)
