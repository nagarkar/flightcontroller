out = instrfind('Status','open');
for i = 1:length(out)
    fclose(out(i));
end