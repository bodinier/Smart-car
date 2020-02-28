function lab = RGB2LABPixel(rgb)   
    rgbl = RGBs2RGBLinearPixel(rgb);
    xyz = RGBLinear2XYZPixel(rgbl);
    lab = XYZ2LABPixel(xyz);
end


function rgbl = RGBs2RGBLinearPixel(rgb)
    T = 0.04045;
    rgbl = zeros(1,3);
    for i = 1:3
        if rgb(i) <T
            rgbl(i) = rgb(i)/12.92; 
        else
            rgbl(i) = ((rgb(i)+0.055)/1.055)^2.4;
        end
    end
end



function xyz = RGBLinear2XYZPixel(rgbl)
    M = [0.4124     0.3576      0.1805;
        0.2126      0.7152      0.0722;
        0.0193     0.1192      0.9505];
    xyz = M*rgbl';
    xyz = xyz';
end


function lab = XYZ2LABPixel(xyz) 
    D65white = [0.9505     1       1.0890];
    xyzn = xyz./D65white;
    lab = [ 116*f(xyzn(2))-16;
             500*(f(xyzn(1))-f(xyzn(2)));
             200*(f(xyzn(2))-f(xyzn(3)))];
end

function nonlinearity = f(t)
    if t>((6/29)^3)
        nonlinearity = t^(1/3);
    else
        nonlinearity  = 7.787*t+16/116;
    end
end

    