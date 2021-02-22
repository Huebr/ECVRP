using VrpSolver, JuMP, ArgParse
 using CPLEX
include("data.jl")
include("model.jl")
include("solution.jl")


function parse_commandline(args_array::Array{String,1}, appfolder::String)
    s = ArgParseSettings(usage="##### VRPSolver #####\n\n" *
	   "  On interactive mode, call main([\"arg1\", ..., \"argn\"])", exit_after_help=false)
    @add_arg_table s begin
        "instance"
            help = "Instance file path"
        "--cfg", "-c"
            help = "Configuration file path"
            default = "$appfolder/../config/ECVRP.cfg"
        "--ub", "-u"
            help = "Upper bound (primal bound)"
            arg_type = Float64
            default = 10000000.0
        "--nclients", "-C"
            help = "Number of clients in instance"
            arg_type = Int
            default = 999
        "--ncharging", "-R"
            help = "Number of charging stations in instance"
            arg_type = Int
            default = 999
        "--bmultiplier", "-M"
            help = "Maximum battery level(multiplier * battery capacity)"
            arg_type = Float64
            default = 1.0
        "--noround", "-r"
            help = "Round the distance matrix"
            action = :store_true
        "--sol", "-s"
            help = "Solution file path."
        "--out", "-o"
            help = "Path to write the solution found"
        "--tikz", "-t"
            help = "Path to write the TikZ figure of the solution found."
        "--nosolve", "-n"
            help = "Does not call the VRPSolver. Only to check or draw a given solution."
            action = :store_true
        "--batch", "-b"
            help = "batch file path"
    end
   return parse_args(args_array, s)
end




function run_ecvrp(app::Dict{String,Any})
    println("Application parameters:")
    for (arg, val) in app
        println("  $arg  =>  $(repr(val))")
    end
    flush(stdout)

   instance_name = split(basename(app["instance"]), ".")[1] * "_C_" * string(app["nclients"]) * "_R_" * string(app["ncharging"])

   data = readECVRPData(app)

    if app["sol"] != nothing
        sol = readsolution(app)
        checksolution(data, sol) # checks the solution feasibility
        app["ub"] = (sol.cost < app["ub"]) ? sol.cost : app["ub"] # update the upper bound if necessary
    end

    solution_found = false
    if !app["nosolve"]
        (model, x) = build_model(data)

         #enum_paths, complete_form = get_complete_formulation(model, app["cfg"])
        #complete_form.solver = CplexSolver() # set MIP solver
        #print_enum_paths(enum_paths)
        #println(complete_form)
        #solve(complete_form,relaxation = true)
        #println("Objective value: $(getobjectivevalue(complete_form))\n")
        
        optimizer = VrpOptimizer(model,app["cfg"], instance_name)
        
        set_cutoff!(optimizer, app["ub"])
        (status, solution_found) = optimize!(optimizer)
        if solution_found
            sol = getsolution(data, optimizer, x, get_objective_value(optimizer), app)
        end
    end

    println("########################################################")
    if solution_found || app["sol"] != nothing # Is there a solution?
        print_routes(sol)
        checksolution(data, sol)
        println("Cost $(sol.cost)")
        if app["out"] != nothing
            writesolution(app["out"], sol)
        end
        if app["tikz"] != nothing
                drawsolution(app["tikz"], data, sol) # write tikz figure
        end
    elseif !app["nosolve"]
        if status == :Optimal
            println("Problem infeasible")
        else
            println("Solution not found")
        end
    end
    println("########################################################")
end

function main(ARGS)
    appfolder = dirname(@__FILE__)
    app = parse_commandline(ARGS, appfolder)
    isnothing(app) && return
    if app["batch"] != nothing
        for line in readlines(app["batch"])
            if isempty(strip(line)) || strip(line)[1] == '#'
                continue
            end
            args_array = [String(s) for s in split(line)]
            app_line = parse_commandline(args_array, appfolder)
            run_ecvrp(app_line)
        end
    else
        run_ecvrp(app)
    end
end

# main()
if isempty(ARGS)
    main(["--help"])
else
    main(ARGS)
end
