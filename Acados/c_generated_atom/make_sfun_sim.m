%
% Copyright (c) The acados authors.
%
% This file is part of acados.
%
% Licensed under the 2-Clause BSD License.

%

SOURCES = { ...
        'acados_sim_solver_sfunction_atom_4ws_nmpc.c', ...
        'acados_sim_solver_atom_4ws_nmpc.c', ...
        'atom_4ws_nmpc_model/atom_4ws_nmpc_expl_ode_fun.c', ...
        'atom_4ws_nmpc_model/atom_4ws_nmpc_expl_vde_forw.c', ...
        'atom_4ws_nmpc_model/atom_4ws_nmpc_expl_vde_adj.c', ...
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

COMPFLAGS = [COMPFLAGS ' -O2'];
CFLAGS = [CFLAGS ' -O2'];


try
    % mex('-v', '-O', CFLAGS, LDFLAGS, COMPFLAGS, COMPDEFINES, INCS{:}, ...
    mex('-O', CFLAGS, LDFLAGS, COMPFLAGS, COMPDEFINES, INCS{:}, ...
        LIB_PATH, LIBS{:}, SOURCES{:}, ...
        '-output', 'acados_sim_solver_sfunction_atom_4ws_nmpc');

catch exception
    disp('make_sfun_sim failed with the following exception:')
    disp(exception);
    disp(exception.message);
    disp('Try adding -v to the mex command above to get more information.')
    keyboard
end


fprintf( [ '\n\nSuccessfully created sfunction:\nacados_sim_solver_sfunction_atom_4ws_nmpc', '.', ...
    eval('mexext')] );


global sfun_sim_input_names
sfun_sim_input_names = {};

%% print note on usage of s-function
fprintf('\n\nNote: Usage of Sfunction is as follows:\n')
input_note = 'Inputs are:\n1) x0, initial state, size [9]\n ';
i_in = 2;
sfun_sim_input_names = [sfun_sim_input_names; 'x0 [9]'];
input_note = strcat(input_note, num2str(i_in), ') u, size [2]\n ');
i_in = i_in + 1;
sfun_sim_input_names = [sfun_sim_input_names; 'u [2]'];
input_note = strcat(input_note, num2str(i_in), ') parameters, size [17]\n ');
i_in = i_in + 1;
sfun_sim_input_names = [sfun_sim_input_names; 'p [17]'];

fprintf(input_note)

disp(' ')

global sfun_sim_output_names
sfun_sim_output_names = {};

output_note = strcat('Outputs are:\n', ...
                '1) x1 - simulated state, size [9]\n');
sfun_sim_output_names = [sfun_sim_output_names; 'x1 [9]'];

fprintf(output_note)


% create the Simulink block for the integrator
modelName = 'atom_4ws_nmpc_sim_solver_simulink_block';
new_system(modelName);
open_system(modelName);

blockPath = [modelName '/atom_4ws_nmpc_sim_solver'];
add_block('simulink/User-Defined Functions/S-Function', blockPath);
set_param(blockPath, 'FunctionName', 'acados_sim_solver_sfunction_atom_4ws_nmpc');

Simulink.Mask.create(blockPath);


display_name = 'atom_4ws_nmpc acados sim';
input_labels = '';
for i = 1:length(sfun_sim_input_names)
	input_labels = [input_labels, sprintf('port_label(''input'', %d, ''%s'')\n', i, sfun_sim_input_names{i})];
end
output_labels = '';
for i = 1:length(sfun_sim_output_names)
	output_labels = [output_labels, sprintf('port_label(''output'', %d, ''%s'')\n', i, sfun_sim_output_names{i})];
end
mask_str = [input_labels, output_labels, sprintf('disp(''%s'')', display_name)];

mask = Simulink.Mask.get(blockPath);
mask.Display = mask_str;

save_system(modelName);
close_system(modelName);
disp([newline, 'Created the sim solver Simulink block in: ', modelName])
