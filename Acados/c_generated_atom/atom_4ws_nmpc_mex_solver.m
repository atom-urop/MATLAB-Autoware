%
% Copyright (c) The acados authors.
%
% This file is part of acados.
%
% Licensed under the 2-Clause BSD License.

%

classdef atom_4ws_nmpc_mex_solver < handle

    properties
        C_ocp
        N
        name
        code_gen_dir
    end % properties



    methods

        % constructor
        function obj = atom_4ws_nmpc_mex_solver(solver_creation_opts)
            if solver_creation_opts.compile_mex_wrapper
                make_mex_atom_4ws_nmpc();
            end
            obj.C_ocp = acados_mex_create_atom_4ws_nmpc();
            % to have path to destructor when changing directory
            addpath('.')
            obj.N = 15;
            obj.name = 'atom_4ws_nmpc';
        end

        % destructor
        function delete(obj)
            disp("delete template...");
            if ~isempty(obj.C_ocp)
                acados_mex_free_atom_4ws_nmpc(obj.C_ocp);
            end
            disp("done.");
        end

        function solve(obj)
            acados_mex_solve_atom_4ws_nmpc(obj.C_ocp);
        end

        function status = custom_update(obj, data)
            status = acados_mex_custom_update_atom_4ws_nmpc(obj.C_ocp, data);
        end

        function P = get_zoRO_Pk_matrices(obj, varargin)
            % P = ocp. get_zoRO_Pk_matrices()       -> returns 1x(N+1) cell of P^k matrices
            % P = ocp.get_zoRO_Pk_matrices(stage)  -> returns [nx x nx] matrix for stage
            if nargin == 1
                P = acados_mex_get_zoRO_Pk_atom_4ws_nmpc(obj.C_ocp);
            else
                P = acados_mex_get_zoRO_Pk_atom_4ws_nmpc(obj.C_ocp, varargin{1});
            end
        end

        function set(varargin)
            % usage:
            % ocp.set(field, value) - value is a flattened concatenation of a trajectory of values
            % ocp.set(field, value, stage) - set the value for the given field at a single stage
            % ocp.set(field, value, stage_0, stage_e) - set the value for the given field at a range of stages, 0-based indexing, stage_e is not included
            obj = varargin{1};
            field = varargin{2};
            value = varargin{3};
            if ~isa(field, 'char')
                error('field must be a char vector, use '' ''');
            end
            if nargin==3
                acados_mex_set_atom_4ws_nmpc(obj.C_ocp, field, value);
            elseif nargin==4
                stage = varargin{4};
                acados_mex_set_atom_4ws_nmpc(obj.C_ocp, field, value, stage);
            elseif nargin==5
                stage_0 = varargin{4};
                stage_e = varargin{5};
                acados_mex_set_atom_4ws_nmpc(obj.C_ocp, field, value, stage_0, stage_e);
            else
                error('acados_ocp.set: wrong number of input arguments, 2 to 4 are allowed.\n');
            end
        end


        function set_params_sparse(varargin)
            % usage:
            % ocp.set_params_sparse(idx_values, param_values, Optional[stage])
            % updates the parameters with indices idx_values (0 based) at stage with the new values new_p_values.
            % if stage is not provided, sparse parameter update is performed for all stages.
            obj = varargin{1};
            idx_values = varargin{2};
            param_values = varargin{3};
            value = [idx_values(:), param_values(:)];
            field = 'params_sparse';

            if ~isa(field, 'char')
                error('field must be a char vector, use '' ''');
            end
            if nargin==3
                acados_mex_set_atom_4ws_nmpc(obj.C_ocp, field, value);
            elseif nargin==4
                stage = varargin{4};
                acados_mex_set_atom_4ws_nmpc(obj.C_ocp, field, value, stage);
            else
                error('acados_ocp.set_params_sparse: wrong number of input arguments (3 to 4 allowed)');
            end
        end

        function value = get_cost(obj)
            value = ocp_get(obj.C_ocp, 'cost_value');
        end

        function value = evaluate_constraints_and_get_violation(obj)
            value = ocp_get(obj.C_ocp, 'constraint_violation');
        end

        function dump_last_qp_to_json(obj, varargin)
            %%% Dumps the latest QP data into a json file
            %%% param1: filename: if not set, use model_name + '_QP.json'
            if nargin>=2
                filename = varargin{1};
            else
                filename = [obj.name, '_QP.json'];
            end
            ocp_dump_qp(obj.C_ocp, filename);
        end

        function eval_param_sens(obj, field, stage, index)
            ocp_eval_param_sens(obj.C_ocp, field, stage, index);
        end

        function value = get(varargin)
            % usage:
            % obj.get(field, value, [stage])
            obj = varargin{1};
            field = varargin{2};
            if ~isa(field, 'char')
                error('field must be a char vector, use '' ''');
            end
            if any(strfind('sens', field))
                error('field sens* (sensitivities of optimal solution) not yet supported for templated MEX.')
            end

            if nargin==2
                value = ocp_get(obj.C_ocp, field);
            elseif nargin==3
                stage = varargin{3};
                value = ocp_get(obj.C_ocp, field, stage);
            elseif nargin==4
                stage = varargin{3};
                iteration = varargin{4};
                value = ocp_get(obj.C_ocp, field, stage, iteration);
            else
                error('acados_ocp.get: wrong number of input arguments (1, 2 or 3 allowed)');
            end


            % make symmetric (only lower triangular stored internally)
            if strcmp('qp_Q', field) || strcmp('qp_R', field)
                if iscell(value)
                    for i=1:length(value)
                        if length(value{i}) > 1
                            value{i} = tril(value{i}) + tril(value{i}, -1)';
                        end
                    end
                else
                    if length(value) > 1
                        value = tril(value) + tril(value, -1)';
                    end
                end
            end
        end

        function [] = reset(varargin)
            % usage:
            % obj.reset()
            % obj.reset(reset_qp_solver_mem, reset_numerical_values, reset_solver_options, reset_x_to_x0_bar)
            % additional flags are optional, default values are 1,0,0,0
            obj = varargin{1};

            if nargin > 5
                error('reset expects at most 4 flags: reset_qp_solver_mem, reset_numerical_values, reset_solver_options, reset_x_to_x0_bar');
            end

            flags = [1, 0, 0, 0];
            for i=2:nargin
                if ~isscalar(varargin{i}) || ~(varargin{i} == 0 || varargin{i} == 1)
                    error('reset flags must be numeric (0 or 1)');
                end
                flags(i-1) = varargin{i};
            end

            acados_mex_set_atom_4ws_nmpc(obj.C_ocp, 'reset', flags);
        end


        % print
        function print(varargin)
            if nargin < 2
                field = 'stat';
            else
                field = varargin{2};
            end

            obj = varargin{1};

            if strcmp(field, 'stat')
                stat = obj.get('stat');
                fprintf('\niter\tres_stat\tres_eq\t\tres_ineq\tres_comp\tqp_stat\tqp_iter\talpha');
                if size(stat,2)>8
                    fprintf('\tqp_res_stat\tqp_res_eq\tqp_res_ineq\tqp_res_comp');
                end
                fprintf('\n');
                for jj=1:size(stat,1)
                    fprintf('%d\t%e\t%e\t%e\t%e\t%d\t%d\t%e', stat(jj,1), stat(jj,2), stat(jj,3), stat(jj,4), stat(jj,5), stat(jj,6), stat(jj,7), stat(jj, 8));
                    if size(stat,2)>8
                        fprintf('\t%e\t%e\t%e\t%e', stat(jj,9), stat(jj,10), stat(jj,11), stat(jj,12));
                    end
                    fprintf('\n');
                end
                fprintf('\n');

            else
                fprintf('unsupported field in function print of acados_ocp.print, got %s', field);
                keyboard
            end

        end

    end % methods

end % class

