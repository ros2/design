# Markdown checks

`npm` is a nodejs-based package manager for javascript modules. `npm` reads a package.json file and installs modules as defined there.
It can install files to system, to user home, or to a local folder ('`node_modules`'). Softlonks to executable scripts are installed to `node_modules/.bin`.

`npm` can conveniently be installed via `nvm` (https://github.com/creationix/nvm), or apt-get.

The `package.json` file defines the requirements to be installed via `npm`.

`mdast` (https://github.com/wooorm/mdast) is a javascript based markdown parser / emitter, with additional plugins providing e.g. linting. `mdast` itself can be used to fix style issues, but it may produce bad results when the input is invalid (so manual checks are advised).
The file `.mdastrc` configures the plugins to be used.


Commands:

# Provide a nodejs version in the current env
nvm use 4.0.0

# in the folder with package.json, installs packages with dependencies to local folder
npm install

# check all md files in folder recursively using given config file and options
./node_modules/.bin/mdast ../../articles --frail --no-color -q --config-path ./mdast/mdastrc

# when providing single file, mdast produces 'fixed' markdown
./node_modules/.bin/mdast ../../articles/somefile.md
