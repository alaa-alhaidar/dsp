const btn = document.getElementById("connect");
const raw = document.getElementById("raw");

const rmsInBox  = document.getElementById("rmsInVal");
const rmsOutBox = document.getElementById("rmsOutVal");
const gainBox   = document.getElementById("gainVal");
const modeBox   = document.getElementById("modeVal");

const ctx = document.getElementById("chart").getContext("2d");

const chart = new Chart(ctx, {
  type: "line",
  data: {
    labels: [],
    datasets: [
      {
        label: "RMSin [V]",
        borderColor: "#1f77b4",
        data: [],
        yAxisID: "y",
      },
      {
        label: "RMSout [V]",
        borderColor: "#2ca02c",
        data: [],
        yAxisID: "y",
      },
      {
        label: "Gain [dB]",
        borderColor: "#d62728",
        data: [],
        yAxisID: "y1",
      }
    ]
  },
  options: {
    animation: false,
    scales: {
      y: {
        position: "left",
        title: { display: true, text: "RMS [V]" }
      },
      y1: {
        position: "right",
        title: { display: true, text: "Gain [dB]" },
        grid: { drawOnChartArea: false }
      }
    }
  }
});

btn.onclick = async () => {
  const port = await navigator.serial.requestPort();
  await port.open({ baudRate: 115200 });

  const reader = port.readable.getReader();
  const decoder = new TextDecoder();

  let buffer = "";
  let t = 0;

  while (true) {
    const { value, done } = await reader.read();
    if (done) break;

    buffer += decoder.decode(value);
    const lines = buffer.split("\n");
    buffer = lines.pop();

    for (const line of lines) {
      raw.textContent += line + "\n";

      // Passend zu deiner printf()-Zeile:
      // Mode=0 | ... | RMSin=0.171V | RMSout=0.049V | Gain= -10.9 dB
      const match = line.match(
        /Mode=(\d).*RMSin=([\d.]+)V.*RMSout=([\d.]+)V.*Gain=\s*([-\d.]+)\s*dB/
      );

      if (match) {
        const mode   = parseInt(match[1]);
        const rmsIn  = parseFloat(match[2]);
        const rmsOut = parseFloat(match[3]);
        const gain   = parseFloat(match[4]);

        // Boxen aktualisieren
        rmsInBox.textContent  = rmsIn.toFixed(3);
        rmsOutBox.textContent = rmsOut.toFixed(3);
        gainBox.textContent   = gain.toFixed(1);
        modeBox.textContent   = (mode === 0) ? "−10 dB" : "−35 dB";

        // Chart aktualisieren
        chart.data.labels.push(t++);
        chart.data.datasets[0].data.push(rmsIn);
        chart.data.datasets[1].data.push(rmsOut);
        chart.data.datasets[2].data.push(gain);

        if (chart.data.labels.length > 100) {
          chart.data.labels.shift();
          chart.data.datasets.forEach(ds => ds.data.shift());
        }

        chart.update();
      }
    }
  }
};
