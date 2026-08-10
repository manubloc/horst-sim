/* Browser-Shim für Node-'module' (Emscripten-Loader importiert createRequire,
 * nutzt es aber nur im Node-Zweig). */
export const createRequire = () => {
  const r = () => { throw new Error('createRequire ist im Browser nicht verfügbar'); };
  r.resolve = r;
  return r;
};
export default { createRequire };
