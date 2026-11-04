function PrintLutYaml()
here     = fileparts(mfilename('fullpath'));   % folder of this file
repoRoot = fileparts(fileparts(here));          % repository root (two levels up)
S = load(fullfile(repoRoot, 'LUT', 'LUT_rr_computation.mat'));
emitRow('k_ref_lut',   S.k_ref_LUT);
emitRow('rr_lut',      S.rr_LUT);
emitRow('delta_f_lut', S.delta_f_LUT);
end

function emitRow(name, v)
fprintf('    %s: [', name);
fprintf('%.6f, ', v(1:end-1));
fprintf('%.6f]\n', v(end));
end