%
% Copyright (c) The acados authors.
%
% This file is part of acados.
%
% Licensed under the 2-Clause BSD License.

%




  
  
  
  
  
  
  
  
  
  
  
  
  
  




SOURCES = { ...
            'atom_4ws_nmpc_model/atom_4ws_nmpc_expl_ode_fun.c', ...
            'atom_4ws_nmpc_model/atom_4ws_nmpc_expl_vde_forw.c', ...
            'atom_4ws_nmpc_model/atom_4ws_nmpc_expl_vde_adj.c', ...
            'atom_4ws_nmpc_constraints/atom_4ws_nmpc_constr_h_0_fun.c', ...
            'atom_4ws_nmpc_constraints/atom_4ws_nmpc_constr_h_0_fun_jac_uxt_zt.c', ...
            'atom_4ws_nmpc_constraints/atom_4ws_nmpc_constr_h_fun.c', ...
            'atom_4ws_nmpc_constraints/atom_4ws_nmpc_constr_h_fun_jac_uxt_zt.c', ...
            'atom_4ws_nmpc_constraints/atom_4ws_nmpc_constr_h_e_fun.c', ...
            'atom_4ws_nmpc_constraints/atom_4ws_nmpc_constr_h_e_fun_jac_uxt_zt.c', ...
            'acados_solver_sfunction_atom_4ws_nmpc.c', ...
            'acados_solver_atom_4ws_nmpc.c'
          };

INC_PATH = '/home/husain5/acados/include';

INCS = {['-I', fullfile(INC_PATH, 'blasfeo', 'include')], ...
        ['-I', fullfile(INC_PATH, 'hpipm', 'include')], ...
        ['-I', fullfile(INC_PATH, 'acados')], ...
        ['-I', fullfile(INC_PATH)]};



CFLAGS = 'CFLAGS=$CFLAGS';
LDFLAGS = 'LDFLAGS=$LDFLAGS';
COMPFLAGS = 'COMPFLAGS=$COMPFLAGS';
COMPDEFINES = 'COMPDEFINES=$COMPDEFINES';



LIB_PATH = ['-L', fullfile('/home/husain5/acados/lib')];

LIBS = {'-lacados', '-lhpipm', '-lblasfeo'};

% acados linking libraries and flags
    
    

COMPFLAGS = [COMPFLAGS ' -O2'];
CFLAGS = [CFLAGS ' -O2'];

try
    %     mex('-v', '-O', CFLAGS, LDFLAGS, COMPFLAGS, COMPDEFINES, INCS{:}, ...
    mex('-O', CFLAGS, LDFLAGS, COMPFLAGS, COMPDEFINES, INCS{:}, ...
            LIB_PATH, LIBS{:}, SOURCES{:}, ...
            '-output', 'acados_solver_sfunction_atom_4ws_nmpc' );
catch exception
    disp('make_sfun failed with the following exception:')
    disp(exception);
    disp(exception.message );
    disp('Try adding -v to the mex command above to get more information.')
    keyboard
end

fprintf( [ '\n\nSuccessfully created sfunction:\nacados_solver_sfunction_atom_4ws_nmpc', '.', ...
    eval('mexext')] );


%% print note on usage of s-function, and create I/O port names vectors
fprintf('\n\nNote: Usage of Sfunction is as follows:\n')
input_note = 'Inputs are:\n';
i_in = 1;

global sfun_input_names
sfun_input_names = {};
input_note = strcat(input_note, num2str(i_in), ') lbx_0 - lower bound on x for stage 0,',...
                    ' size [9]\n ');
sfun_input_names = [sfun_input_names; 'lbx_0 [9]'];
i_in = i_in + 1;
input_note = strcat(input_note, num2str(i_in), ') ubx_0 - upper bound on x for stage 0,',...
                    ' size [9]\n ');
sfun_input_names = [sfun_input_names; 'ubx_0 [9]'];
i_in = i_in + 1;
input_note = strcat(input_note, num2str(i_in), ') parameters - concatenated for all stages 0 to N,',...
                    ' size [272]\n ');
sfun_input_names = [sfun_input_names; 'parameter_traj [272]'];
i_in = i_in + 1;
input_note = strcat(input_note, num2str(i_in), ') y_ref_0 - size [7]\n ');
sfun_input_names = [sfun_input_names; 'y_ref_0 [7]'];
i_in = i_in + 1;



input_note = strcat(input_note, num2str(i_in), ') y_ref - concatenated for stages 1 to N-1,',...
                    ' size [98]\n ');
sfun_input_names = [sfun_input_names; 'y_ref [98]'];
i_in = i_in + 1;
input_note = strcat(input_note, num2str(i_in), ') y_ref_e - size [5]\n ');
sfun_input_names = [sfun_input_names; 'y_ref_e [5]'];
i_in = i_in + 1;



input_note = strcat(input_note, num2str(i_in), ') x_init - initialization of x for all stages, size [144]\n ');
sfun_input_names = [sfun_input_names; 'x_init [144]'];
i_in = i_in + 1;
input_note = strcat(input_note, num2str(i_in), ') u_init - initialization of u for stages 0 to N-1, size [30]\n ');
sfun_input_names = [sfun_input_names; 'u_init [30]'];
i_in = i_in + 1;
input_note = strcat(input_note, num2str(i_in), ') pi_init - initialization of pi for stages 0 to N-1, size [135]\n ');
sfun_input_names = [sfun_input_names; 'pi_init [135]'];
i_in = i_in + 1;

fprintf(input_note)

disp(' ')

output_note = 'Outputs are:\n';
i_out = 0;

global sfun_output_names
sfun_output_names = {};
i_out = i_out + 1;
output_note = strcat(output_note, num2str(i_out), ') u0, control input at node 0, size [2]\n ');
sfun_output_names = [sfun_output_names; 'u0 [2]'];
i_out = i_out + 1;
output_note = strcat(output_note, num2str(i_out), ') acados solver status (0 = SUCCESS)\n ');
sfun_output_names = [sfun_output_names; 'solver_status'];
i_out = i_out + 1;
output_note = strcat(output_note, num2str(i_out), ') CPU time\n ');
sfun_output_names = [sfun_output_names; 'CPU_time'];
i_out = i_out + 1;
output_note = strcat(output_note, num2str(i_out), ') SQP iterations\n ');
sfun_output_names = [sfun_output_names; 'sqp_iter'];



fprintf(output_note)
