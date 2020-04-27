function [] = create_ephysenet_inifile()
fid = fopen( 'ephysenet.ini', 'wt' );
text = fileread('ephysenet_original.ini');
fwrite(fid,text);
fclose(fid);
end

