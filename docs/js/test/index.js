/*
 * Entry point for `npm test` (= `node --test docs/js/test/`). Node 22 runs a directory given to
 * --test as a module, i.e. this file; older Node versions instead run every test file in the
 * directory (parity.test.js, options.test.js, demo-data.test.js), and this file then does nothing
 * (no duplicate run).
 */
'use strict';
if (!/index\.js$/.test(process.argv[1] || '')) {
  require('./parity.test.js');
  require('./options.test.js');
  require('./demo-data.test.js');
}
