// @ts-check

/** @type {import('@docusaurus/types').PluginModule} */
const createPlugin = (context, options) => {
  return {
    name: 'docusaurus-plugin-chapter-buttons',

    injectHtmlTags() {
      return {
        headTags: [
          {
            tagName: 'script',
            attributes: {
              src: '/js/chapter-buttons.js',
              type: 'text/javascript',
            },
          },
        ],
      };
    },
  };
};

module.exports = createPlugin;