import express from "express";
import fs from "fs";
import path from "path";
import cors from "cors";
import bodyParser from "body-parser";
import { fileURLToPath } from "url"; // Required to handle __dirname in ES modules

const app = express();
const PORT = 5000;

// Manually define __dirname since it's not available in ES Modules
const __filename = fileURLToPath(import.meta.url);
const __dirname = path.dirname(__filename);

app.use(cors());
app.use(bodyParser.json());

// API to save a new component
app.post("/save-component", async (req, res) => {
    const { componentName, code } = req.body;
    if (!componentName || !code) {
        return res.status(400).json({ error: "Component name and code are required!" });
    }

    const filePath = path.join(__dirname, "src", "components", `${componentName}.jsx`);

    // Ensure "components" directory exists
    try {
        await fs.promises.mkdir(path.join(__dirname, "src", "components"), { recursive: true });
        await fs.promises.writeFile(filePath, code);
        res.json({ message: `Component ${componentName}.jsx saved successfully!` });
    } catch (err) {
        console.error("Error writing file:", err);
        res.status(500).json({ error: "Failed to save component." });
    }
});

// Start the server
app.listen(PORT, () => {
    console.log(`Server running on http://localhost:${PORT}`);
});


