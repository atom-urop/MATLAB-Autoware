function PrintLutYaml()
S = load('/home/husain5/MATLAB-Autoware/LUT/LUT_rr_computation.mat');
emitRow('k_ref_lut',   S.k_ref_LUT);
emitRow('rr_lut',      S.rr_LUT);
emitRow('delta_f_lut', S.delta_f_LUT);
end

function emitRow(name, v)
fprintf('    %s: [', name);
fprintf('%.6f, ', v(1:end-1));
fprintf('%.6f]\n', v(end));
end