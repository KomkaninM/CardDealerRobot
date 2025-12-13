self.declare_parameter("env_file", "")
env_file = self.get_parameter("env_file").get_parameter_value().string_value

if env_file:
    load_dotenv(env_file)
else:
    # fallback: หา .env ที่ workspace root (เผื่อไว้)
    load_dotenv(Path.cwd() / ".env")
