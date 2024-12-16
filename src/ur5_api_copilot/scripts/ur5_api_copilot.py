import getpass
import os

from flask import Flask, request, jsonify
from langchain_google_genai import ChatGoogleGenerativeAI

app = Flask(__name__)

# Get API key
if "GOOGLE_API_KEY" not in os.environ:
    os.environ["GOOGLE_API_KEY"] = getpass.getpass("Enter your Google AI API key: ")

# Initialize the LLM model
llm = ChatGoogleGenerativeAI(
    model="gemini-1.5-pro",
    temperature=0,
    max_tokens=1000000,
    timeout=None,
    max_retries=2,
    # other params...
)

@app.route('/suggest_json', methods=['POST'])
def suggest_code():
    """
    Endpoint to generate JSON request suggestions using an LLM model.
    """
    try:
        # Parse the user input
        data = request.get_json()
        prompt = data.get("prompt")
        max_length = data.get("max_length", 150)

        if not prompt:
            return jsonify({"error": "Prompt is required"}), 400

        messages = [
            (
                "system",
                """You are a helpful assistant that analyzes ROS .srv file and creates suggestions for JSON request body. 
                Pose1 and Pose2 have three members. Create a JSON request body.""",
            ),
            ("human", prompt),
        ]

        # Generate code suggestions
        ai_msg = llm.invoke(messages)

        return jsonify({
            "prompt": prompt,
            "suggestions": ai_msg.content
        }), 200

    except Exception as e:
        return jsonify({"error": str(e)}), 500

@app.route('/explain_route', methods=['POST'])
def explain_route():
    """
    Explain API routes using an LLM model.
    """
    try:
        # Parse the user input
        data = request.get_json()
        prompt = data.get("prompt")
        max_length = data.get("max_length", 150)

        if not prompt:
            return jsonify({"error": "Prompt is required"}), 400

        messages = [
            (
                "system",
                "You are a helpful assistant that explains API. Explaine route meaning.",
            ),
            ("human", prompt),
        ]

        # Generate code suggestions
        ai_msg = llm.invoke(messages)

        return jsonify({
            "prompt": prompt,
            "explanation": ai_msg.content
        }), 200

    except Exception as e:
        return jsonify({"error": str(e)}), 500

if __name__ == "__main__":
    app.run(debug=True, port=6000)
