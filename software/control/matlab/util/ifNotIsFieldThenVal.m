function struct_object = ifNotIsFieldThenVal(struct_object,fieldname,default_value)
    if ~isfield(struct_object,fieldname)
      struct_object.(fieldname) = default_value;
    end
end
