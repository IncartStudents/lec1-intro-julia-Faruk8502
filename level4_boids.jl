module Boids
using Plots
using Random

mutable struct Boid
    x::Float64
    y::Float64
    vx::Float64
    vy::Float64
end

mutable struct WorldState
    boids::Vector{Boid}
    height::Float64
    width::Float64
    function WorldState(n_boids, width, height)
        boids = Vector{Boid}()
        rng = MersenneTwister(1234) # Для воспроизводимости
        
        for _ in 1:n_boids
            x = rand(rng) * width
            y = rand(rng) * height
            vx = randn(rng) * 0.1
            vy = randn(rng) * 0.1
            push!(boids, Boid(x, y, vx, vy))
        end
        
        new(boids, width, height)
    end
end

function update!(state::WorldState)
    for boid in state.boids
        # Обновляем позицию
        boid.x += boid.vx
        boid.y += boid.vy
        
        # Обработка границ - если boid выходит за границы, появляется с другой стороны
        if boid.x < 0
            boid.x = state.width
        elseif boid.x > state.width
            boid.x = 0
        end
        
        if boid.y < 0
            boid.y = state.height
        elseif boid.y > state.height
            boid.y = 0
        end
    end
    return nothing
end

function main(ARGS)
    w = 30.0
    h = 30.0
    n_boids = 10

    state = WorldState(n_boids, w, h)

    anim = @animate for time = 1:100
        update!(state)
        scatter([(b.x, b.y) for b in state.boids], 
                xlim = (0, state.width), 
                ylim = (0, state.height),
                legend = false,
                title = "Boids Simulation",
                markersize = 5,
                markercolor = :blue)
    end
    gif(anim, "boids.gif", fps = 10)
end

export main

end

using .Boids
Boids.main("")
