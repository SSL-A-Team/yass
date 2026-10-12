use schemars::schema_for;
use yass::config::Config;



fn main() {
    let schema = schema_for!(Config);
    let yaml_schema = serde_yaml::to_string(&schema).expect("Failed to serialize schema to YAML");
    println!("{}", yaml_schema);
}
