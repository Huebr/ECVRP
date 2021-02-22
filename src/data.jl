import Unicode

mutable struct Vertex
    id_vertex::Int
    str_id::String
    pos_x::Float64
    pos_y::Float64
    demand::Float64
 end

# Undirected graph
mutable struct InputGraph
    V′::Array{Vertex} # set of vertices
    E::Array{Tuple{Int64,Int64}} # set of edges
    cost::Dict{Tuple{Int64,Int64},Float64} # cost for each edge
    energy_cost::Dict{Tuple{Int64,Int64},Float64} # cost for each edge
end

mutable struct DataECVRP
    G′::InputGraph
    Clients::Array{Int64} # block nodes
    ChargingStations::Array{Int64} # white nodes
    Q::Float64 # Vehicle load capacity
    E_max::Float64 # Maximum energy capacity
    E_min::Float64 # minimum energy capacity
    r::Float64 # energy comsuption rate
    round::Bool # Is the distance matrix rounded?
end

vertices(data::DataECVRP) = [i.id_vertex for i in data.G′.V′[1:end]] # return set of vertices

# Euclidian distance only
function distance(data::DataECVRP, arc::Tuple{Int64, Int64})
    e = (arc[1] < arc[2]) ? arc : (arc[2],arc[1])
    if haskey(data.G′.cost, e) # use already calculated value
       return data.G′.cost[e]
    else
       u, v = arc
       vertices = data.G′.V′ 
       # array <vertices> is indexed from 1 (depot is vertices[1], customer 1 is vertices[2], and so on) 
       x_sq = (vertices[v+1].pos_x - vertices[u+1].pos_x)^2
       y_sq = (vertices[v+1].pos_y - vertices[u+1].pos_y)^2
       if data.round
          return floor(sqrt(x_sq + y_sq) + 0.5)
       end
       return sqrt(x_sq + y_sq)
    end
 end



contains(p, s) = findnext(s, p, 1) != nothing

function readECVRPData(app::Dict{String,Any})

    str = Unicode.normalize(read(app["instance"], String); stripcc=true)
    breaks_in = [' ';'/'; '\n']
    aux = split(str, breaks_in; limit=0, keepempty=false)

    G′ = InputGraph([], [], Dict(),Dict())
    data = DataECVRP(G′, [], [], 0.0,0.0,0.0,0.0, !app["noround"])

    cost_aux, insType_aux = [], " "
    dim = 0
    base_row(row,idx) = row*8+idx+1 #numbercols=8 to ignore first line add 1

    n_clients = app["nclients"]
    n_charging_stations =  app["ncharging"]
    total_elements = 1 + n_clients+ n_charging_stations



    #deposit in row 1
    for i in 1:total_elements
        v = Vertex(0," ", 0, 0, 0)
        v.id_vertex = i-1 # deposit position zero
        v.str_id = aux[(base_row(i,1))]
        v.pos_x = parse(Float64, aux[base_row(i,3)])
        v.pos_y = parse(Float64, aux[base_row(i,4)])
        v.demand = parse(Float64, aux[base_row(i,5)])
        push!(G′.V′, v) # add v in the vertex array
       if contains(aux[base_row(i,2)],"f")
         push!(data.ChargingStations,i-1)
       elseif contains(aux[base_row(i,2)],"c")
         push!(data.Clients,i-1)
       end
    end

    R′ = vcat([0],data.ChargingStations)
     # need touse hard coded positions
     data.E_max =app["bmultiplier"]*parse(Float64, aux[base_row(total_elements+1,6)])
     data.E_min = 0
     data.Q = parse(Float64 ,aux[base_row(total_elements+1,11)])
     data.r = parse(Float64 ,aux[base_row(total_elements+1,16)])



    for i in data.Clients # setting the arcs between source, sink, and black vertices
        # source -> i(black)
        for j in R′
            e = (i < j) ? (i, j) : (j, i)
            push!(G′.E, e) # add edge e
            data.G′.cost[e] = distance(data, e)
            data.G′.energy_cost[e] = data.r*distance(data, e)
        end
    end
     for i in data.Clients # setting the arcs between source, sink, and black vertices
        # source -> i(black)
         for j in data.Clients
             if (i < j)
                e = (i, j)
                push!(G′.E, e) # add edge e
                data.G′.cost[e] = distance(data, e)
                data.G′.energy_cost[e] = data.r*distance(data, e)
             end
          end
      end
    
    return data
end


edges(data::DataECVRP) = data.G′.E # return set of edges
c(data,e) = data.G′.cost[e] # cost of the edge e
ec(data,e) = data.G′.energy_cost[e]
d(data,e) = (e[1] != e[2]) ? data.G′.cost[e] : 0.0 # cost of the edge e
d(data::DataECVRP,i::Int64) = (i in data.Clients) ? data.G′.V′[i+1].demand : 0.0

dimension(data::DataECVRP) = length(data.G′.V′) # return number of vertices
nb_vertices(data::DataECVRP) = length(vertices(data))

