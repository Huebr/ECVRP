mutable struct Solution
    cost::Union{Int,Float64}
    routes::Array{Array{Int}}
end


function getsolution(data::DataECVRP, optimizer::VrpOptimizer, x, objval, app::Dict{String,Any})
    dim = dimension(data)
    V = [i for i in 0:dim-1] # set of vertices of the input graph G′
    R = data.ChargingStations
    R′ = vcat([0],R)
    A = [(i,j) for i in V for j in V if !((i in  R′) & (j in  R′)) & (i!=j)]
    adj_list = [[] for i in 1:dim] 
    n_v = get_number_of_positive_paths(optimizer)
    println("Number of vehicles $(n_v)")
    #println("---------------x values-----------------------------")
    for a in A
       val = get_value(optimizer, x[a])
       #println((a,val))
       if val > 0.5
          push!(adj_list[a[1]+1], a[2])
       end
     #  println("x[$a]=$val")
    end
    #println("---------------------------------------------------")
    visited, routes = [false for i in 1:dim], []
    for j in  R′
        for i in adj_list[j+1]
            r, prev = [j], j 
            push!(r, i)
            visited[i+1] = true
            length(adj_list[i+1]) != 1 && error("Problem trying to recover the route from the x values. "*
                                                "Customer $i has $(length(adj_list[i+1])) outcoming arcs.")
            next = adj_list[i+1][1]
            maxit, it = dim, 0
            while next != j && it < maxit
                if(next in data.Clients)
                    length(adj_list[next+1]) != 1 && error("Problem trying to recover the route from the x values. "* 
                    "Customer $next has $(length(adj_list[next+1])) outcoming arcs.")
                    push!(r, next)
                    visited[next+1] = true
                    next = adj_list[next+1][1]
                else
                    valid_list = filter(x -> visited[x+1] == false, adj_list[next+1])
                    length(valid_list) ==0 && error("Problem trying to recover the route from the x values. "*
                    "ChargingStation $next has $(length(adj_list[next+1])) outcoming arcs.")
                    push!(r, next)
                    adj_list[next+1] = valid_list[2:end]
                    next = valid_list[1]
                end
                it += 1
            end
            (it == maxit) && error("Problem trying to recover the route from the x values. "*
                                    "Some route can not be recovered because the return to depot is never reached")
            push!(r, j)
            push!(routes, r)
        end 
    end

    !isempty(findall(a->a==false,visited[2+length(data.ChargingStations):end])) && error("Problem trying to recover the route from the x values. "*
                              "At least one customer was not visited or there are subtours in the solution x. $(visited[2+length(data.ChargingStations):end])")
    #merging routes testing
    for r in routes
        if(r[1]!=0)
            pathid = 1
           for i in 1:n_v
               if(get_value(optimizer,x[ed(r[1],r[2])],i)>0.5)
                  pathid=i
               end
           end

           for id in 1:length(routes)
            if routes[id][1]==0 &&(get_value(optimizer,x[ed(routes[id][1],routes[id][2])],pathid)>0.5)
                i = 1
                while (routes[id][i]!=r[1])
                    i=i+1
                end
                #merging routes
                a = vcat(routes[id][1:i-1],r)
                a = vcat(a,routes[id][i+1:end])
                routes[id] = a
                break
            end
           end
        end
    end   

    routes = filter(a->a[1]==0,routes)
    if !app["noround"]
        objval = trunc(Int, round(objval))
    end
    return Solution(objval, routes)
 end


function print_routes(solution)
    for (i, r) in enumerate(solution.routes)
        print("Route #$i: ")
        for j in r
            print("$j ")
        end
        println()
    end
end

# checks the feasiblity of a solution
ed(i, j) = i < j ? (i, j) : (j, i)


function checksolution(data::DataECVRP, solution)
    dim, Q = dimension(data), data.Q
    visits = [0 for i in 1:dim]
    sum_cost = 0.0
    E_max = data.E_max
    E_min = data.E_min
    C = data.Clients
    R = data.ChargingStations
    R′ = vcat([0],R) 
    for (i, r) in enumerate(solution.routes)
        sum_demand, battery_level, prev = 0, E_max, r[1]
        #cummulative =0.0
        for j in r[2:end]
            visits[j+1] += 1
            (visits[j+1] > 1& j in C) && error("Customer $j was visited more than once")
            sum_cost += c(data, ed(prev, j))
            #cummulative = (prev in R′) ?  ec(data,ed(j,prev)) : cummulative + ec(data,ed(j,prev))
            #println("$(cummulative) -> $(j)->")
            battery_level= (prev in R′) ? E_max - ec(data,ed(j,prev)) : battery_level - ec(data,ed(prev,j))
            #println("$(battery_level)")
            (battery_level < E_min) && error("Route is violating the limit of energy. The battery level is not bounded $(battery_level) in [$E_min,$E_max]")
            sum_demand += d(data,  j)
            (sum_demand > Q) && error("Route #$i is violating the capacity constraint. Sum of the demands is $(sum_demand) and Q is $Q")
            prev = j
        end
    end
    !isempty(filter(a -> a == 0 , visits[2+length(data.ChargingStations):end])) && error("The following vertices were not visited: $(filter(a -> a == 0, visits[2+length(data.ChargingStations):end]))")
    (abs(solution.cost - sum_cost) > 0.001) && error("Cost calculated from the routes ($sum_cost) is different from that passed as" *
                                                                                                  " argument ($(solution.cost)).")
end

# read solution from file (CVRPLIB format)
function readsolution(app::Dict{String,Any})
    str = read(app["sol"], String)
    breaks_in = [' '; ':'; '\n';'\t';'\r']
    aux = split(str, breaks_in; limit=0, keepempty=false)
    sol = Solution(0, [])
    j = 3
    while j <= length(aux)
        r = []
        while j <= length(aux)
            push!(r, parse(Int, aux[j]))
            j += 1
            if contains(lowercase(aux[j]), "cost") || contains(lowercase(aux[j]), "route")
                break
            end
        end
        push!(sol.routes, r)
        if contains(lowercase(aux[j]), "cost")
            if app["noround"]
                sol.cost = parse(Float64, aux[j + 1])
            else
                sol.cost = parse(Int, aux[j + 1])
            end
            return sol
        end
        j += 2 # skip "Route" and "#j:" elements
    end
    error("The solution file was not read successfully.")
    return sol
end


# write solution in a file
function writesolution(solpath, solution)
    open(solpath, "w") do f
        for (i, r) in enumerate(solution.routes)
            write(f, "Route #$i: ")
            for j in r
                write(f, "$j ")
            end
            write(f, "\n")
        end
        write(f, "Cost $(solution.cost)\n")
    end
end

# write solution as TikZ figure (.tex)
function drawsolution(tikzpath, data, solution)
    open(tikzpath, "w") do f
        write(f, "\\documentclass[crop,tikz]{standalone}\n\\begin{document}\n")
        # get limits to draw
        pos_x_vals = [i.pos_x for i in data.G′.V′]
        pos_y_vals = [i.pos_y for i in data.G′.V′]
        scale_fac = 1 / (max(maximum(pos_x_vals), maximum(pos_y_vals)) / 10)


        write(f, "\\begin{tikzpicture}[thick, scale=1, every node/.style={scale=0.3}]\n")
            for i in data.G′.V′
                x_plot = scale_fac * i.pos_x
                y_plot = scale_fac * i.pos_y
                if i.id_vertex in data.ChargingStations
                    write(f, "\t\\node[draw, line width=0.1mm, circle, fill=black, inner sep=0.05cm, text=white] (v$(i.id_vertex)) at ($(x_plot),$(y_plot)) {\\footnotesize $(i.id_vertex)};\n")
                else
                    write(f, "\t\\node[draw, line width=0.1mm, circle, fill=white, inner sep=0.05cm] (v$(i.id_vertex)) at ($(x_plot),$(y_plot)) {\\footnotesize $(i.id_vertex)};\n")
                end
            end

            for r in solution.routes
                prev = r[1]
                first = r[1]
                edge_style = "-,line width=0.8pt"
                for i in r[1:end]
                    e = (prev, i)
                    write(f, "\t\\draw[$(edge_style)] (v$(e[1])) -- (v$(e[2]));\n")
                    prev = i
                end
                write(f, "\t\\draw[$(edge_style)] (v$(first)) -- (v$(prev));\n")
            end

            write(f, "\\end{tikzpicture}\n")
            write(f, "\\end{document}\n")
    end
end
