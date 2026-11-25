import { init, use } from 'echarts/core';
import { LineChart } from 'echarts/charts';
import { GridComponent } from 'echarts/components';
import { CanvasRenderer } from 'echarts/renderers';
use([LineChart, GridComponent, CanvasRenderer]);
export { init };
export const chartTheme = { color: ['#ff7532'], backgroundColor: 'transparent', textStyle: { color: '#c7c0b9' } };
