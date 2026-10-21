import {build} from 'esbuild';
await build({entryPoints:['src/MotionScene.jsx'],outfile:'dist/scene.bundle.js',bundle:true,format:'esm',platform:'browser',target:['chrome100','edge100'],jsx:'automatic',minify:true,legalComments:'eof',define:{'process.env.NODE_ENV':'"production"'}});
console.log('Built self-contained React motion scene.');
