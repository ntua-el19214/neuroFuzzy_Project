function tyre_choice(c)
    path = fileparts(mfilename('fullpath'));
    if c == "R20"
        rmpath(path+"/LC0")
    elseif c=="LC0"
        rmpath(path+"/R20")
    else
        ME = MException('Vehicle:noSuchTyre', 'Tyre %s not found',c);
        throw(ME)
    end
end
