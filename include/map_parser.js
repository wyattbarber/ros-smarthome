/*
    Parses a map object from a set of name-value pairs.
    
    @param names array of variable names
    @param values array of variable values, with values[i] corresponding to names[i]
*/
exports.parse = function (names, values) {
    let obj = {};
    let i = 0;
    names.forEach(name => {
        let value;
        if (!isNaN(values[i])) { // Check if value is a numeric value
            value = Number(values[i]);
        }
        else if (values[i] == "true" || values[i] == "false") { // Check if value is boolean
            value = (values[i] == "true");
        }
        else { // value is string
            value = values[i];
        }
        obj[name] = value;
    });
    return obj;
}
