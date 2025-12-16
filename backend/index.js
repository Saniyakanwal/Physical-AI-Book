import express from "express";
import cors from "cors";
import dotenv from "dotenv";

dotenv.config();

const app = express();
app.use(cors());
app.use(express.json());

app.get("/", (req, res) => {
  res.json({ message: "Physical AI Backend running 🚀" });
});

app.post("/chat", async (req, res) => {
  const { message } = req.body;

  // yahan baad mein RAG + AI connect karoge
  res.json({
    reply: `You said: ${message}`,
  });
});

const PORT = process.env.PORT || 8000;
app.listen(PORT, () => {
  console.log(`Backend running on http://localhost:${PORT}`);
});
