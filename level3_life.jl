
module GameOfLife
using Plots

mutable struct Life
    current_frame::Matrix{Int}
    next_frame::Matrix{Int}
end

function step!(state::Life)
    curr = state.current_frame
    next = state.next_frame
    n, m = size(curr)
    for i in 1:n, j in 1:m
        # Считаем количество живых соседей с учетом тора (граничные условия)
        neighbors = 0
        for di in -1:1, dj in -1:1
            if di == 0 && dj == 0
                continue  # Пропускаем текущую клетку
            end
            ni = mod1(i + di, n)
            nj = mod1(j + dj, m)
            neighbors += curr[ni, nj]
        end
        
        # Применяем правила игры "Жизнь"
        if curr[i, j] == 1
            # Клетка жива
            if neighbors == 2 || neighbors == 3
                next[i, j] = 1  # Остается живой
            else
                next[i, j] = 0  # Умирает
            end
        else
            # Клетка мертва
            if neighbors == 3
                next[i, j] = 1  # Оживает
            else
                next[i, j] = 0  # Остается мертвой
            end
        end
    end
    # Обновляем текущий кадр
    state.current_frame, state.next_frame = state.next_frame, state.current_frame

    return nothing
end

function (@main)(ARGS)
    n = 30
    m = 30
    init = rand(0:1, n, m)

    game = Life(init, zeros(n, m))

    anim = @animate for time = 1:100
        step!(game)
        cr = game.current_frame
        heatmap(cr)
    end
    gif(anim, "life.gif", fps = 10)
end

export main

end

using .GameOfLife
GameOfLife.main("")
