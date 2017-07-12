 function XYZ = MagnetometerBiasEstimator( filename)
    fid = fopen(filename, 'r', 'native', 'UTF-8');
    A = fread(fid, 'double');
    fclose(fid);    
    XYZ = reshape(A, [3, length(A)/3])';   
 end