function p3dump()
%P3DUMP  Writes the current layout (data\Phase3\warehouse_layout.mat) and the scene file list into one text file, so it can be sent / pasted.
%   p3dump   ->  data\Phase3\out\Diagnostics\layout_dump.txt
P = phase3Paths(); if ~isfolder(P.diag), mkdir(P.diag); end
S = load(P.layout); fn = fieldnames(S); f = fullfile(P.diag,'layout_dump.txt'); fid = fopen(f,'w');
for i = 1:numel(fn)
    v = S.(fn{i});
    if isstruct(v)
        w = fieldnames(v);
        for q = 1:numel(w)
            x = v.(w{q});
            if isnumeric(x) || islogical(x)
                if numel(x) > 400, fprintf(fid,'%s.%s : %s array %s (not printed)\n',fn{i},w{q},class(x),mat2str(size(x)));
                else, fprintf(fid,'%s.%s = %s\n',fn{i},w{q},mat2str(x,5)); end
            elseif ischar(x), fprintf(fid,'%s.%s = ''%s''\n',fn{i},w{q},x);
            else, fprintf(fid,'%s.%s : %s %s\n',fn{i},w{q},class(x),mat2str(size(x))); end
        end
    end
end
fclose(fid);
fprintf('Layout written to %s\n',f);
end
