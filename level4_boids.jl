module Boids
using Plots
using Random
using LinearAlgebra

mutable struct Boid
    x::Float64
    y::Float64
    vx::Float64
    vy::Float64
    ax::Float64
    ay::Float64
    max_speed::Float64
    max_force::Float64
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
            ax = 0
            ay = 0
            max_speed = 1.0
            max_force = 0.05
            push!(boids, Boid(x, y, vx, vy, ax, ay, max_speed, max_force))
        end
        
        new(boids, width, height)
    end
end
# Правило 1: избегание столкновений
function separation(boid::Boid, others, desired_separation=5.0)
    steer = zeros(2)
    count = 0
    boid_position = [boid.x, boid.y]
    for other in others
        other_position = [other.x, other.y]
        d = norm(boid_position - other_position)
        if 0 < d < desired_separation
            diff = boid_position - other_position
            diff = diff / d  # Нормализация
            steer += diff
            count += 1
        end
    end
    
    if count > 0
        steer = steer / count
        if norm(steer) > 0
            steer = steer / norm(steer) * boid.max_speed
            steer = steer - [boid.vx, boid.vy]
            steer = steer .* (boid.max_force / norm(steer))
        end
    end
    
    return steer
end

# Правило 2: согласование направления
function alignment(boid::Boid, others, alignment_distance=5.0)
    avg_velocity = zeros(2)
    count = 0
    boid_position = [boid.x, boid.y]
    boid_velocity = [boid.vx, boid.vy]
    for other in others
        other_position = [other.x, other.y]
        other_velocity = [other.vx, other.vy]
        d = norm(boid_position - other_position)
        if 0 < d < alignment_distance
            avg_velocity += other_velocity
            count += 1
        end
    end
    
    if count > 0
        avg_velocity = avg_velocity / count
        if norm(avg_velocity) > 0
            avg_velocity = avg_velocity / norm(avg_velocity) * boid.max_speed
            steer = avg_velocity - boid_velocity
            steer = steer .* (boid.max_force / norm(steer))
            return steer
        end
    end
    
    return zeros(2)
end

# Правило 3: движение к центру группы
function cohesion(boid::Boid, others, cohesion_distance=8.0)
    center_of_mass = zeros(2)
    count = 0
    boid_position = [boid.x, boid.y]
    boid_velocity = [boid.vx, boid.vy]
    for other in others
        other_position = [other.x, other.y]
        d = norm(boid_position - other_position)
        if 0 < d < cohesion_distance
            center_of_mass += other_position
            count += 1
        end
    end
    
    if count > 0
        center_of_mass = center_of_mass / count
        desired_direction = center_of_mass - boid_position
        if norm(desired_direction) > 0
            desired_direction = desired_direction / norm(desired_direction) * boid.max_speed
            steer = desired_direction - boid_velocity
            steer = steer .* (boid.max_force / norm(steer))
            return steer
        end
    end
    
    return zeros(2)
end

function update!(state::WorldState)
    for boid in state.boids
        neighbors = [n for n in state.boids if n ≠ boid]
        # distances = map(b -> sqrt((b.x - xcor)^2 + (b.y - ycor)^2), neighbors)
        # near_distances = filter(x -> x <= radius, distances)

        sep = separation(boid, neighbors) * 1.0  # Весовой коэффициент
        ali = alignment(boid, neighbors) * 1.0 # Весовой коэффициент
        coh = cohesion(boid, neighbors) * 1.0 # Весовой коэффициент

        sep = separation(boid, neighbors)

        boid.ax = sep[1] + ali[1] + coh[1]
        boid.ay = sep[2] + ali[2] + coh[2]

        boid.vx += boid.ax
        boid.vy += boid.ay

        # if norm([boid.vx, boid.vy]) > boid.max_speed
        #     boid.vx = boid.vx / norm([boid.vx, boid.vy]) * boid.max_speed
        #     boid.vy = boid.vy / norm([boid.vx, boid.vy]) * boid.max_speed
        # end

        # Обновляем позицию
        boid.x += boid.vx
        boid.y += boid.vy
        
        # Обработка границ: если boid выходит за границы, появляется с другой стороны
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

# Функция для вычисления формы треугольника, ориентированного по скорости
function triangle_shape(boid::Boid, size=0.5)
    # Нормализуем скорость для определения направления
    v = [boid.vx, boid.vy]
    if norm(v) ≈ 0
        v = [1.0, 0.0]  # Направление по умолчанию, если скорость нулевая
    else
        v = v / norm(v)
    end
    
    # Перпендикулярный вектор
    perp = [-v[2], v[1]]
    
    # Вершины треугольника (носик в направлении скорости)
    p1 = [boid.x, boid.y] + v * size
    p2 = [boid.x, boid.y] - v * size/2 + perp * size/2
    p3 = [boid.x, boid.y] - v * size/2 - perp * size/2
    
    # Возвращаем координаты для Plots
    ([p1[1], p2[1], p3[1]], [p1[2], p2[2], p3[2]])
end

function main(ARGS)
    w = 30.0
    h = 30.0
    n_boids = 10

    state = WorldState(n_boids, w, h)

    anim = @animate for time = 1:1000
        update!(state)
        # scatter([(b.x, b.y) for b in state.boids], 
        #         xlim = (0, state.width), 
        #         ylim = (0, state.height),
        #         legend = false,
        #         title = "Boids Simulation",
        #         markersize = 5,
        #         markercolor = :blue)
        # Создаем пустой график
        plot(xlim=(0, w), ylim=(0, h), aspect_ratio=:equal, legend=false)

        # Рисуем каждого boid в виде треугольника
        for boid in state.boids
            xs, ys = triangle_shape(boid)
            plot!(xs, ys, seriestype=:shape, fillcolor=:blue, linecolor=:black)
        end
    end
    gif(anim, "boids.gif", fps = 10)
end

export main end

using .Boids
Boids.main("")
