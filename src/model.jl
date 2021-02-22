

function build_model(data::DataECVRP)

    E = edges(data) # set of edges of the input graph G′
    n = nb_vertices(data)
    V = [i for i in 0:n-1] # set of vertices of the input graph G′
    #V′ = [i for i in 0:n] # V ⋃ {0}, where 0 is a dummy vertex

    Q = data.Q #  Maximum Load
    E_max = data.E_max
    E_min = data.E_min
    max_battery = E_max - E_min #max battery level represents that the battery is fully discharged to minimum level viable
    C = data.Clients # Set of white vertices
    C′ = vcat([0],C) #C ⋃ {0}, where 0 is a depot vertex
    R = data.ChargingStations # Set of black vertices
    R′ = vcat([0],R) #R ⋃ {0}, where 0 is a depot vertex
    V′ = data.G′.V′
    ed(i, j) = (i < j) ? (i, j) : (j, i)
    
    Ec = [(i,j) for i in V for j in V if !((i in  R′) & (j in  R′)) & (i!=j)]#complete graph
    # Formulation
     ecvrp = VrpModel()


    @variable(ecvrp.formulation, x[e in Ec], Int)
        
    
    @objective(ecvrp.formulation, Min, sum(c(data, ed(e[1],e[2])) * x[e] for e in Ec))
    @constraint(ecvrp.formulation, cons2[j in C], sum(x[(i,j)] for i in V if i != j) == 1.0)
    #@constraint(ecvrp.formulation, cons3, sum(c(data,ed(i,j))*x[ed(i,j)] for i in C for j in V if i!=j) <= sum(E_max*x[ed(i,j)] for i in R′ for j in C))
    #@constraint(ecvrp.formulation, cons4[j in R], sum(x[(i,j)] for i in C)<=1.0  )
    # println(ecvrp.formulation)

    # Build the model directed graph G=(V,A)
    function build_graph()

        v_source = v_sink = 0
        L = 1 
        U = length(C) # max and min number of paths is equal to number of black nodes
        artificial_vertex = [ i for i in n:n+length(R)-1]
        inviable_edges  = []
        
        #better bound
        for i in C

            if(ec(data,(0,i)) + minimum([ec(data,ed(i,r)) for r in R′]) > E_max - E_min)
               U = U - 1
               push!(inviable_edges,(0,i))
               push!(inviable_edges,(i,0))
            end
        end


        # node ids of G from 0 to |V|
        V= vcat(V,artificial_vertex)
        G = VrpGraph(ecvrp, V, v_source, v_sink, (L, U))
        # resourves, R = R_M = {1,2} = {cap_res_id, energy_res_id}}
        cap_res_id = add_resource!(G, main=true)
        energy_res_id = add_resource!(G, main=false)
        for i in V
            l_i, u_i = 0.0, Float64(Q) # accumulated resource consumption interval [l_i, u_i] for the vertex i
            set_resource_bounds!(G, i, cap_res_id, l_i, u_i)

            l_i, u_i = E_min, Float64(max_battery)
            set_resource_bounds!(G, i,  energy_res_id, l_i, u_i)
        end
        
        # Build set of arcs A from E′ (two arcs for each edge (i,j))
        for i in C # setting the arcs between source, sink, and black vertices
            # source -> i(black)
            for j in R′
                if(c(data,ed(i,j))<=E_max-E_min && ! ((i,j) in inviable_edges))
                arc_id = add_arc!(G, i, j)

                add_arc_var_mapping!(G, arc_id, x[(i,j)])
                set_arc_consumption!(G, arc_id, cap_res_id, (d(data,i)+d(data,j))/2)
                set_arc_consumption!(G, arc_id, energy_res_id, ec(data,ed(i,j)))
                # i(black) -> sink
               
                 arc_id = add_arc!(G, (j!=0) ? j+n-1 : j, i)
                 #arc_id = add_arc!(G,j,i)
                 add_arc_var_mapping!(G, arc_id, x[(j,i)])
                 set_arc_consumption!(G, arc_id, cap_res_id, (d(data,i)+d(data,j))/2)
                 set_arc_consumption!(G, arc_id, energy_res_id, ec(data,ed(i,j)))
                 #set_arc_resource_bounds!(G,arc_id,  energy_res_id, ec(data,ed(i,j)), E_max)
                 #set_arc_consumption!(G, arc_id, energy_res_id, -E_max)
                else
                    println("removing inviable edges : $((i,j))")
                end
            end
        end
        for j in R
            arc_id = add_arc!(G, j, j+n-1)
            set_arc_consumption!(G, arc_id, cap_res_id, 0.0)
            set_arc_consumption!(G, arc_id, energy_res_id, -E_max)
        end
        for i in C # setting the arcs between clients.

            for j in C
                if (i < j)

                    if(ec(data,ed(i,j))<=E_max-E_min)
                      arc_id = add_arc!(G, i, j)
                      add_arc_var_mapping!(G, arc_id, x[(i,j)])
                      set_arc_consumption!(G, arc_id, cap_res_id, (d(data,i)+d(data,j))/2)
                      set_arc_consumption!(G, arc_id, energy_res_id, ec(data,(i,j)))


                      arc_id = add_arc!(G, j, i)
                      add_arc_var_mapping!(G, arc_id, x[(j,i)])
                      set_arc_consumption!(G, arc_id, cap_res_id, (d(data,i)+d(data,j))/2)
                      set_arc_consumption!(G, arc_id, energy_res_id, ec(data,(i,j)))
                    else
                        println("removing inviable edges : $((i,j))")
                    end
                end
            end
        end
        return G
    end

    G = build_graph()
    add_graph!(ecvrp, G)
    #println(G)
    
    #M = vcat(R,C)

    set_vertex_packing_sets!(ecvrp, [[(G, i)] for i in 1:n-1])
    #set_additional_vertex_elementarity_sets!(ecvrp, [(G,[i]) for i in R])
    #define_elementarity_sets_distance_matrix!(ecvrp, G, [[( !(issubset([i,j],R)) ? d(data,ed(i, j)) : 1e7 )  for i in M] for j in M])
    define_elementarity_sets_distance_matrix!(ecvrp, G, [[d(data,ed(i, j))  for i in C] for j in C])

    add_capacity_cut_separator!(ecvrp, [ ([(G, i)], d(data,i)) for i in C], Float64(Q))
    set_branching_priority!(ecvrp, "x", 1)


    return (ecvrp, x)
end
