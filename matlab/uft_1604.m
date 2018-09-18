function [meta, data] = uft_1604()
    meta.date = '20180917/';
    meta.run = '001';
    
    

    meta.dataroot = '/home/benjamin/ros/data/';
    try [data.fta] = loaddatadotm('fta', meta); catch; disp(['    ** Issue loading pid data']); end

end

function [out] = loaddatadotm(in, meta)
    here = pwd;
    root = [meta.dataroot meta.date meta.run '/'];
    mat_file = [in '_' meta.run '.mat'];
    m_file = [in '_' meta.run '.m'];
    cd(root);
    
    if exist(mat_file)
        disp(['    Loading ' mat_file ' ...'])
        load(mat_file);
    elseif exist(m_file, 'file')
        
        disp(['    Loading ' root m_file ' ...'])
        try
            eval([in '_prealloc_' meta.run])
        catch
%             debugprint(['     ' in ': no preallocation found'])
        end
        eval([in '_' meta.run])
        if exist(in, 'var')
            vprint(['     Saving ' root mat_file ' ...'])
            save([root mat_file], in)
        end
    end
    out = eval(in);
    cd(here);
end