const {
  addDecoratorsLegacy,
  override,
  disableEsLint,
  addBabelPlugin,
} = require('customize-cra')

module.exports = {
  webpack: override(
    addDecoratorsLegacy(),
    disableEsLint(),
    addBabelPlugin(['module:@import-meta-env/babel', { 'example': '.env.example' }])
  ),
}
