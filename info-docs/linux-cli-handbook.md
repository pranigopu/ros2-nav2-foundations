<h1>LINUX CLI HANDBOOK</h1>

---

**Contents**:

- [`chmod`](#chmod)
- [`source`](#source)

---

**NOTE**: Commands = Programs or scripts callable via names (in the CLI)

# `chmod`
`chmod` => "change mode"

Command to change access permissions of a file or directory.

---

**Notable usage** (replace `...` with the file's path):

- `chmod +x ...`: Give the file permission to execute instructions <br> *Relevant for scripts and binary files*

# `source`
Command to execute an executable/script file by giving its path.

---

See [`source` Command in Linux](./source-command-in-linux) for more.

---

**Notable usage**:

- `source ~/.bashrc`
	- `.bashrc` is a bash script executed at the start of new terminal session
	- It is stored in the home directory, referred to by `~/`
	
# Preventing outputs from script
> **References**:
>
> - [*Preventing Terminal Print from Bash Scripts in Linux*, **squash.io**](https://www.squash.io/preventing-terminal-print-from-bash-scripts-in-linux/)
> - [*What is /Dev/Null in Linux?*, **GeeksForGeeks.org**](https://www.geeksforgeeks.org/what-is-dev-null-in-linux/)

# `head` and `tail` commands
> **Reference**: [*The head and tail commands in Linux*, **Baeldung.com**](https://www.baeldung.com/linux/head-tail-commands)